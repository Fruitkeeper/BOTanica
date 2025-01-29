#!/usr/bin/env python3
import rospy
from robomaster import robot
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from threading import Thread
import numpy as np

class RoboMasterDriver:
    def __init__(self):
        # Initialize robot
        self._robot = robot.Robot()
        try:
            conn_mode = rospy.get_param("~connection_mode", "WIFI")
            if conn_mode == "WIFI":
                self._robot.initialize(conn_type="sta")
            elif conn_mode == "USB":
                self._robot.initialize(conn_type="rndis")
            else:
                rospy.logerr(f"Unknown connection mode {conn_mode}")
                return
        except Exception as e:
            rospy.logerr(f"Failed to connect to robot: {e}")
            return
            
        rospy.loginfo("Connected to RoboMaster successfully")
        
        # Set robot mode and brake
        self._robot.set_robot_mode(mode=robot.FREE)  # Use FREE mode
        self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        
        # Initialize publishers
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        
        # Initialize subscribers
        self._cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self._cmd_vel_callback, queue_size=1)
        self._robot.chassis.sub_position(cs=1, freq=50, callback=self._position_callback)
        
        # Track last command time
        self._last_cmd_time = rospy.Time.now()
        self._cmd_timeout = rospy.Duration(0.5)  # Stop if no commands for 0.5 seconds

    def _cmd_vel_callback(self, msg):
        """Velocity command handler"""
        try:
            # Update command time
            self._last_cmd_time = rospy.Time.now()
            
            # Check if this is a zero command
            if abs(msg.linear.x) < 0.01 and abs(msg.linear.y) < 0.01 and abs(msg.angular.z) < 0.01:
                # Apply brake
                self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                return

            # Convert velocities to wheel speeds
            # Assuming X configuration and wheel radius of ~0.05m
            wheel_radius = 0.05  # meters
            robot_width = 0.32   # meters (distance between wheels)
            
            # Calculate individual wheel speeds
            v_x = msg.linear.x
            v_y = -msg.linear.y
            omega = -msg.angular.z
            
            # Convert to wheel velocities (RPM)
            scale = 60.0 / (2.0 * np.pi * wheel_radius)  # Convert m/s to RPM
            w1 = (v_x + v_y + omega * robot_width) * scale  # Front right
            w2 = (v_x - v_y + omega * robot_width) * scale  # Front left
            w3 = (v_x - v_y - omega * robot_width) * scale  # Rear right
            w4 = (v_x + v_y - omega * robot_width) * scale  # Rear left
            
            # Send wheel speeds directly
            self._robot.chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4)
            
        except Exception as e:
            rospy.logwarn(f"Failed to send velocity command: {e}")
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    def _position_callback(self, data):
        """Simple position publisher"""
        # Check command timeout
        if (rospy.Time.now() - self._last_cmd_time) > self._cmd_timeout:
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            
        x, y, z = data
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = -y
        msg.pose.pose.position.z = 0.0
        self._odom_pub.publish(msg)

    def shutdown(self):
        """Clean shutdown"""
        try:
            self._robot.chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            rospy.sleep(0.1)
            self._robot.close()
        except Exception as e:
            rospy.logerr(f"Error during shutdown: {e}")

if __name__ == "__main__":
    rospy.init_node("robomaster_driver")
    driver = RoboMasterDriver()
    rospy.on_shutdown(driver.shutdown)
    rospy.spin()