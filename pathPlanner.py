import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_info_publisher.msg import SensorData
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class LightPathPlanner:
    def __init__(self):
        rospy.init_node('light_path_planner', anonymous=True)

        # Publishers and subscribers
        rospy.Subscriber('sensor_data1', SensorData, self.sensor_data_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turn_pub = rospy.Publisher('turn_and_ping', Bool, queue_size=10)

        # Variables for sensor data, angles, and odometry
        self.sensor_data = None
        self.light_values = {i: [0, 0] for i in range(0, 360, 45)}
        self.current_angle = 0
        self.target_angle = 0
        self.new_data_received = False
        self.turn_completed = False
        self.kp = 1.0  # Proportional control gain
        self.tolerance = 0.02  # Radians tolerance
        self.min_angular_speed = 0.05
        self.max_angular_speed = 1.0

        # Ensure robot starts at angle 0
        self.align_to_zero()

    def sensor_data_callback(self, msg):
        """Callback to process sensor data."""
        self.sensor_data = msg
        self.new_data_received = True

    def odometry_callback(self, msg):
        """Callback to track odometry and determine when the turn is complete."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_angle = yaw  # Update the current yaw angle

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def rotate_to_angle(self, angle):
        """Rotate the robot to a specific angle using tuned proportional control."""
        self.target_angle = math.radians(angle)  # Convert target angle to radians
        self.turn_completed = False
        rospy.loginfo(f"Rotating to {angle} degrees.")

        rate = rospy.Rate(10)
        command = Twist()

        while not rospy.is_shutdown():
            # Calculate angular error
            angular_error = self.normalize_angle(self.target_angle - self.current_angle)

            # Check if the robot is aligned
            if abs(angular_error) < self.tolerance:
                self.turn_completed = True
                break

            # Proportional control
            angular_speed = max(min(self.kp * angular_error, self.max_angular_speed), -self.max_angular_speed)

            # Ensure minimum speed is met
            if 0 < abs(angular_speed) < self.min_angular_speed:
                angular_speed = self.min_angular_speed if angular_speed > 0 else -self.min_angular_speed

            # Command the robot
            command.angular.z = angular_speed
            self.cmd_pub.publish(command)
            rate.sleep()

        # Stop the robot after completing the turn
        command.angular.z = 0
        self.cmd_pub.publish(command)

        if self.turn_completed:
            rospy.loginfo(f"Rotation to {angle} degrees completed.")
        else:
            rospy.logwarn("Rotation timed out or failed.")

        rospy.sleep(0.5)  # Brief pause before the next action

    def align_to_zero(self):
        """Align the robot to angle 0 before starting."""
        rospy.loginfo("Aligning to angle 0 before starting.")
        self.rotate_to_angle(0)

    def ping_sensor(self):
        """Ping the sensor three times and return the average lux values."""
        # Ensure robot is stationary
        command = Twist()
        command.linear.x = 0
        command.angular.z = 0
        self.cmd_pub.publish(command)

        lux_1 = []
        lux_2 = []
        for i in range(3):
            self.new_data_received = False
            self.turn_pub.publish(True)  # Trigger the sensor ping
            rospy.loginfo(f"Pinging sensors (attempt {i + 1}/3).")

            # Wait for sensor data
            timeout = 15  # Increase timeout to 15 seconds
            while not self.new_data_received and timeout > 0 and not rospy.is_shutdown():
                rospy.sleep(0.1)
                timeout -= 0.1

            if self.new_data_received and self.sensor_data:
                lux_1.append(self.sensor_data.light_intensity_1)
                lux_2.append(self.sensor_data.light_intensity_2)
            else:
                rospy.logwarn("No sensor data received. Retrying...")

            rospy.sleep(2)  # Wait for 2 seconds between pings

        # Calculate averages
        avg_lux_1 = sum(lux_1) / len(lux_1) if lux_1 else 0
        avg_lux_2 = sum(lux_2) / len(lux_2) if lux_2 else 0
        rospy.loginfo(f"Avg Lux - Sensor 1: {avg_lux_1}, Sensor 2: {avg_lux_2}")
        return avg_lux_1, avg_lux_2

    def scan_environment(self):
        """Perform a 360° scan and collect averaged light data."""
        for angle in range(0, 360, 45):
            self.rotate_to_angle(angle)

            if not self.turn_completed:
                rospy.logwarn(f"Skipping ping at angle {angle}° due to incomplete turn.")
                continue

            # Ensure the robot is stationary
            rospy.sleep(1)  # Add a brief stabilization delay

            avg_lux_1, avg_lux_2 = self.ping_sensor()

            self.light_values[angle] = [avg_lux_1, avg_lux_2]
            rospy.loginfo(f"Angle {angle}°: Avg Sensor 1 = {avg_lux_1}, Avg Sensor 2 = {avg_lux_2}.")

    def find_max_light_direction(self):
        """Find the angle with the maximum light intensity."""
        max_angle = None
        max_light = 0
        for angle, (lux_1, lux_2) in self.light_values.items():
            max_lux = max(lux_1, lux_2)
            if max_lux > max_light:
                max_light = max_lux
                max_angle = angle

        rospy.loginfo(f"Max light at {max_angle}° with intensity {max_light} lux.")
        return max_angle

    def should_stop(self):
        """Determine if the robot should stop based on light intensity distribution."""
        lux_values = [max(lux_1, lux_2) for lux_1, lux_2 in self.light_values.values()]
        max_lux = max(lux_values)
        min_lux = min(lux_values)

        # High percentage of angles with lux > 600
        high_light_angles = [lux for lux in lux_values if lux > 600]
        high_light_percentage = len(high_light_angles) / len(self.light_values)

        # Check conditions
        if high_light_percentage > 0.7 and (max_lux - min_lux) < 50:
            rospy.loginfo("Bright, uniform area found. Stopping movement.")
            return True
        return False

    def move_forward(self, distance):
        """Move the robot forward by a specific distance."""
        rospy.loginfo(f"Moving forward {distance} meters.")
        command = Twist()
        command.linear.x = 0.2  # Forward speed (adjust as needed)

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        duration = distance / command.linear.x  # Time to move the specified distance

        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
            self.cmd_pub.publish(command)
            rate.sleep()

        # Stop the robot
        command.linear.x = 0
        self.cmd_pub.publish(command)
        rospy.loginfo("Movement complete.")

    def execute(self):
        """Main execution loop."""
        rospy.loginfo("Starting light-based path planning.")
        while not rospy.is_shutdown():
            self.scan_environment()

            # Stop if the conditions for bright area are met
            if self.should_stop():
                rospy.loginfo("Robot has found an optimal bright spot. Stopping.")
                break

            # Find the brightest angle and move toward it
            target_angle = self.find_max_light_direction()
            if target_angle is not None:
                self.rotate_to_angle(target_angle)
                rospy.loginfo(f"Aligning to brightest angle {target_angle}°.")
                self.move_forward(0.5)  # Move forward by 0.5 meters

            rospy.sleep(5)  # Pause before repeating

if __name__ == "__main__":
    try:
        planner = LightPathPlanner()
        planner.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path planner node terminated.")
    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        planner.cmd_pub.publish(twist)
        rospy.loginfo("Robot stopped.")

