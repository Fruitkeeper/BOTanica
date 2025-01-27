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
        self.kp = 0.8  # Adjusted Proportional control gain
        self.tolerance = 0.05  # Radians tolerance
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

    def normalize_angle(self, angle):
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
        start_time = rospy.Time.now()
        timeout = rospy.Duration(10)  # 10-second timeout for rotation

        while not rospy.is_shutdown():
            # Break if timeout is exceeded
            if rospy.Time.now() - start_time > timeout:
                rospy.logwarn(f"Timeout exceeded while turning to {angle}°.")
                break

            # Calculate angular error
            angular_error = self.normalize_angle(self.target_angle - self.current_angle)

            # Check if the robot is aligned
            if abs(angular_error) < self.tolerance:
                self.turn_completed = True
                rospy.loginfo(f"Turn completed for angle {angle}°.")
                break

            # Proportional control
            angular_speed = self.kp * angular_error

            # Ensure angular speed is above the minimum threshold if error > tolerance
            if abs(angular_speed) < self.min_angular_speed and abs(angular_error) >= self.tolerance:
                angular_speed = self.min_angular_speed * (1 if angular_error > 0 else -1)

            # Cap angular speed to the maximum allowed value
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

            # Command the robot
            command.angular.z = angular_speed
            self.cmd_pub.publish(command)
            rospy.loginfo(f"Target: {self.target_angle:.2f}, Current: {self.current_angle:.2f}, Error: {angular_error:.2f}, Speed: {angular_speed:.2f}")
            rate.sleep()

        # Stop the robot after completing the turn
        command.angular.z = 0
        self.cmd_pub.publish(command)

        if not self.turn_completed:
            rospy.logwarn(f"Turn to {angle}° timed out or failed.")
        rospy.sleep(0.5)  # Brief pause before the next action

    def align_to_zero(self):
        """Align the robot to angle 0 before starting."""
        rospy.loginfo("Aligning to angle 0 before starting.")
        self.rotate_to_angle(0)

if __name__ == "__main__":
    try:
        planner = LightPathPlanner()
        planner.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path planner node terminated.")
    except Exception as e:
        rospy.logerr(f"Unexpected error occurred: {e}")
    finally:
        try:
            if 'planner' in locals():
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                planner.cmd_pub.publish(twist)
                rospy.loginfo("Robot stopped.")
        except Exception as e:
            rospy.logerr(f"Failed to stop the robot: {e}")

