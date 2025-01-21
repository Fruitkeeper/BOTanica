#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_info_publisher.msg import SensorData
import time

class LightPathPlanner:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('light_path_planner', anonymous=True)

        # Subscribe to consolidated sensor data
        rospy.Subscriber('sensor_data1', SensorData, self.sensor_data_callback)

        # Publisher for robot movement
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Publisher for triggering infosensor
        self.trigger_pub = rospy.Publisher('turn_and_ping', Bool, queue_size=10)

        # Variables to store sensor data and current angle
        self.sensor_data = None
        self.light_values = []
        self.current_angle = 0
        self.angles = [i * 45 for i in range(8)]  # 0°, 45°, ..., 315°

        # Flags to indicate new data availability
        self.new_data_received = False

    def sensor_data_callback(self, msg):
        """Callback to process sensor data."""
        self.sensor_data = msg
        self.new_data_received = True  # Indicate that new data is available

    def rotate_robot(self, angle):
        """Rotate the robot by a specified angle."""
        twist = Twist()
        twist.angular.z = 0.5  # Set angular velocity
        duration = abs(angle) / 45 * 2  # Approximate time to rotate 45° is 2 seconds
        rospy.loginfo(f"Rotating {angle} degrees.")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop the robot after rotation
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def move_forward(self, duration):
        """Move the robot forward for a given duration."""
        twist = Twist()
        twist.linear.x = 0.5  # Set linear velocity
        rospy.loginfo("Moving forward.")
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop the robot after moving
        twist.linear.x = 0
        self.cmd_pub.publish(twist)

    def scan_environment(self):
        """Perform a 360° scan to find the direction of strongest light."""
        self.light_values = []

        for angle in self.angles:
            # Rotate the robot to the next angle
            self.rotate_robot(45)

            # Trigger infosensor to read data
            rospy.loginfo(f"Requesting sensor data for angle {self.current_angle}°.")
            self.trigger_pub.publish(True)

            # Wait for sensor data to be received
            timeout = 10  # seconds
            while not self.new_data_received and timeout > 0 and not rospy.is_shutdown():
                rospy.sleep(0.1)
                timeout -= 0.1

            if self.new_data_received and self.sensor_data is not None:
                rospy.loginfo(f"Received sensor data at angle {self.current_angle}: Light {self.sensor_data.light_intensity_1} lux")
                # Use the light intensity from the first sensor (or modify as needed)
                self.light_values.append((self.current_angle, self.sensor_data.light_intensity_1))
                self.new_data_received = False  # Reset the flag
            else:
                rospy.logwarn(f"No sensor data received at angle {self.current_angle}. Assuming 0 lux.")
                self.light_values.append((self.current_angle, 0))

            self.current_angle = (self.current_angle + 45) % 360  # Update angle

    def find_max_light_direction(self):
        """Find the angle with the strongest light."""
        if not self.light_values:
            rospy.logwarn("No light values recorded during scan.")
            return None
        max_angle, max_lux = max(self.light_values, key=lambda x: x[1])
        rospy.loginfo(f"Max light intensity {max_lux} lux at angle {max_angle}°.")
        return max_angle

    def align_to_angle(self, target_angle):
        """Align the robot to the specified angle."""
        rotation_angle = (target_angle - self.current_angle) % 360
        if rotation_angle > 180:
            rotation_angle -= 360  # Rotate in the shortest direction
        self.rotate_robot(rotation_angle)
        self.current_angle = target_angle

    def execute(self):
        """Main execution loop."""
        rospy.loginfo("Starting light-based path planning.")
        rate = rospy.Rate(1)  # Run at 1 Hz
        while not rospy.is_shutdown():
            # Perform a 360° scan
            self.scan_environment()

            # Find the direction of strongest light
            target_angle = self.find_max_light_direction()
            if target_angle is not None:
                # Align to the target angle and move forward
                self.align_to_angle(target_angle)
                self.move_forward(5)  # Move forward for 5 seconds
            else:
                rospy.logwarn("No valid light direction found. Retrying...")

            # Wait before the next scan
            rospy.sleep(5)


if __name__ == "__main__":
    try:
        planner = LightPathPlanner()
        planner.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path planner node terminated.")

