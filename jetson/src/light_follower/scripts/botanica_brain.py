#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
BOTanica Brain - Priority-based behavior controller with GVF navigation

Priority 1 (highest): Battery low -> Go to dock, charge until 100%
Priority 2: Soil moisture low -> Go to water station, dose, resume light-seeking
Priority 3 (default): Daytime -> Seek brightest light
         Otherwise: Idle

Navigation is handled by vectorfield_stack (GVF).
This node publishes target paths and monitors arrival.
Light-seeking uses direct cmd_vel control (no GVF needed for rotation/short moves).
"""
import rospy
import cv2
import numpy as np
from datetime import datetime
from enum import Enum

from sensor_msgs.msg import Image, BatteryState
from geometry_msgs.msg import Twist, PoseStamped, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion

# Import custom sensor message
from sensor_publisher.msg import SensorData

# Import vectorfield_stack path message
from distancefield.msg import Path as GVFPath


class State(Enum):
    IDLE = "IDLE"
    LIGHT_SCAN = "LIGHT_SCAN"
    LIGHT_ALIGN = "LIGHT_ALIGN"
    LIGHT_MOVE = "LIGHT_MOVE"
    GO_TO_WATER = "GO_TO_WATER"
    DOSING = "DOSING"
    GO_TO_DOCK = "GO_TO_DOCK"
    CHARGING = "CHARGING"


class NavigationMode(Enum):
    GVF = "GVF"           # Let vectorfield_stack control cmd_vel
    DIRECT = "DIRECT"     # This node controls cmd_vel directly


class BOTanicaBrain:
    # === DEFAULT CONFIGURATION ===
    # (Can be overridden via ROS parameters)

    # Thresholds
    DEFAULT_BATTERY_LOW_THRESHOLD = 0.20   # 20% - go to dock
    DEFAULT_BATTERY_FULL = 1.0             # 100% - leave dock
    DEFAULT_MOISTURE_LOW_THRESHOLD = 30    # 30% - go to water

    # Time-based day detection (24h format)
    DEFAULT_DAY_START_HOUR = 6             # 6 AM
    DEFAULT_DAY_END_HOUR = 20              # 8 PM

    # Default waypoints in OptiTrack frame (x, y)
    DEFAULT_DOCK_COORDS = (0.0, 0.0)
    DEFAULT_WATER_COORDS = (1.0, 1.0)

    # Navigation parameters
    DEFAULT_ARRIVAL_TOLERANCE = 0.15       # meters - how close to be "arrived"

    # Light-seeking parameters
    BRIGHTNESS_SCAN_THRESHOLD = 150
    BRIGHTNESS_MOVE_THRESHOLD = 160
    MIN_BRIGHT_ANGLES = 5
    LIGHT_MOVE_SPEED = 0.1
    BRIGHT_CONFIRM_COUNT = 3

    # Dosing duration
    DEFAULT_DOSE_DURATION = 5.0            # seconds to "water"

    def __init__(self):
        rospy.init_node("botanica_brain")

        # Load configuration from ROS parameters (with defaults)
        self.BATTERY_LOW_THRESHOLD = rospy.get_param("~battery_low_threshold", self.DEFAULT_BATTERY_LOW_THRESHOLD)
        self.BATTERY_FULL = rospy.get_param("~battery_full", self.DEFAULT_BATTERY_FULL)
        self.MOISTURE_LOW_THRESHOLD = rospy.get_param("~moisture_low_threshold", self.DEFAULT_MOISTURE_LOW_THRESHOLD)
        self.DAY_START_HOUR = rospy.get_param("~day_start_hour", self.DEFAULT_DAY_START_HOUR)
        self.DAY_END_HOUR = rospy.get_param("~day_end_hour", self.DEFAULT_DAY_END_HOUR)
        self.ARRIVAL_TOLERANCE = rospy.get_param("~arrival_tolerance", self.DEFAULT_ARRIVAL_TOLERANCE)
        self.DOSE_DURATION = rospy.get_param("~dose_duration", self.DEFAULT_DOSE_DURATION)

        # Load waypoints from ROS parameters
        dock_x = rospy.get_param("~dock_x", self.DEFAULT_DOCK_COORDS[0])
        dock_y = rospy.get_param("~dock_y", self.DEFAULT_DOCK_COORDS[1])
        water_x = rospy.get_param("~water_x", self.DEFAULT_WATER_COORDS[0])
        water_y = rospy.get_param("~water_y", self.DEFAULT_WATER_COORDS[1])
        self.DOCK_COORDS = (dock_x, dock_y)
        self.WATER_COORDS = (water_x, water_y)

        self.bridge = CvBridge()
        self.state = State.IDLE
        self.nav_mode = NavigationMode.DIRECT

        # Sensor data
        self.battery_percent = 1.0     # Start assuming full
        self.soil_moisture = 100       # Start assuming watered
        self.image = None

        # Position data
        self.current_pose = None       # (x, y, yaw) from OptiTrack
        self.current_yaw = 0.0

        # Light-seeking state
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_rotation = 0.0
        self.brightness_log = []
        self.target_yaw = 0.0
        self.move_start_pos = None
        self.bright_counter = 0

        # Navigation state
        self.nav_target = None
        self.gvf_active = False

        # Dosing state
        self.dose_start_time = None

        # === PUBLISHERS ===
        # Direct cmd_vel for light-seeking (scan/align/move)
        self.cmd_pub = rospy.Publisher("/cmd_vel_direct", Twist, queue_size=10)

        # Path publisher for vectorfield_stack GVF navigation
        self.path_pub = rospy.Publisher("/gvf/path", GVFPath, queue_size=1)

        # Mux control: switch between GVF and direct control
        # True = use GVF cmd_vel, False = use direct cmd_vel
        self.nav_mode_pub = rospy.Publisher("/nav_mode_gvf", Bool, queue_size=1)

        # === SUBSCRIBERS ===
        # Battery from RoboMaster via Pi
        rospy.Subscriber("/battery", BatteryState, self.battery_callback)

        # Soil moisture from BLE sensor
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)

        # Position from OptiTrack (adjust topic/type as needed)
        rospy.Subscriber("/natnet_ros/umh_5/pose", PoseStamped, self.pose_callback)

        # Camera for light detection
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Odometry as backup / for yaw
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.loginfo("BOTanica Brain initialized")
        rospy.loginfo(f"  Battery threshold: {self.BATTERY_LOW_THRESHOLD*100}%")
        rospy.loginfo(f"  Moisture threshold: {self.MOISTURE_LOW_THRESHOLD}%")
        rospy.loginfo(f"  Day hours: {self.DAY_START_HOUR}:00 - {self.DAY_END_HOUR}:00")
        rospy.loginfo(f"  Dock coords: {self.DOCK_COORDS}")
        rospy.loginfo(f"  Water coords: {self.WATER_COORDS}")

        # Main control loop at 10Hz
        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.spin()

    # === CALLBACKS ===

    def battery_callback(self, msg):
        self.battery_percent = msg.percentage  # 0.0 - 1.0

    def sensor_callback(self, msg):
        self.soil_moisture = msg.moisture  # 0-100%

    def pose_callback(self, msg):
        """OptiTrack pose callback"""
        pos = msg.pose.position
        orient = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.current_pose = (pos.x, pos.y, yaw)
        self.current_yaw = yaw

    def odom_callback(self, msg):
        """Backup odometry from RoboMaster"""
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        # Only use if no OptiTrack pose
        if self.current_pose is None:
            pos = msg.pose.pose.position
            self.current_pose = (pos.x, pos.y, yaw)
        self.current_yaw = yaw

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Image error: {e}")

    # === UTILITY FUNCTIONS ===

    def is_daytime(self):
        hour = datetime.now().hour
        return self.DAY_START_HOUR <= hour < self.DAY_END_HOUR

    def angle_diff(self, a, b):
        d = a - b
        while d > np.pi:
            d -= 2 * np.pi
        while d < -np.pi:
            d += 2 * np.pi
        return d

    def distance_to(self, target):
        if self.current_pose is None:
            return float('inf')
        return np.hypot(self.current_pose[0] - target[0],
                        self.current_pose[1] - target[1])

    def get_brightness(self):
        if self.image is None:
            return 0
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (21, 21), 0)
        return np.mean(blurred)

    # === NAVIGATION CONTROL ===

    def set_nav_mode(self, mode):
        """Switch between GVF and direct control"""
        self.nav_mode = mode
        msg = Bool()
        msg.data = (mode == NavigationMode.GVF)
        self.nav_mode_pub.publish(msg)
        rospy.loginfo(f"Navigation mode: {mode.value}")

    def publish_direct_cmd(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publish direct velocity command (for light-seeking)"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def stop(self):
        """Stop all movement"""
        self.publish_direct_cmd(0, 0, 0)

    def start_gvf_navigation(self, target):
        """
        Start GVF navigation to target waypoint.
        Publishes a path from current position to target.
        vectorfield_stack will handle the actual navigation.
        """
        if self.current_pose is None:
            rospy.logwarn("Cannot start GVF navigation: no pose data")
            return

        self.nav_target = target
        self.set_nav_mode(NavigationMode.GVF)

        # Create path message for vectorfield_stack
        path_msg = GVFPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"  # or your OptiTrack frame

        # Path: current position -> target
        # vectorfield_stack will create a smooth vector field
        start_point = Point32()
        start_point.x = self.current_pose[0]
        start_point.y = self.current_pose[1]
        start_point.z = 0.0

        end_point = Point32()
        end_point.x = target[0]
        end_point.y = target[1]
        end_point.z = 0.0

        path_msg.path.points = [start_point, end_point]
        path_msg.closed_path_flag = False
        path_msg.insert_n_points = 10      # Interpolate for smoother path
        path_msg.filter_path_n_average = 3  # Smooth the path

        self.path_pub.publish(path_msg)
        self.gvf_active = True

        rospy.loginfo(f"GVF navigation started: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) -> ({target[0]:.2f}, {target[1]:.2f})")

    def check_arrival(self):
        """Check if robot has arrived at navigation target"""
        if self.nav_target is None:
            return False
        return self.distance_to(self.nav_target) < self.ARRIVAL_TOLERANCE

    def stop_gvf_navigation(self):
        """Stop GVF navigation and switch to direct control"""
        self.gvf_active = False
        self.nav_target = None
        self.set_nav_mode(NavigationMode.DIRECT)
        self.stop()

    # === MAIN UPDATE LOOP ===

    def update(self, _):
        # === PRIORITY CHECKS (run every cycle) ===

        # P1: Battery critical - override everything except charging
        if self.battery_percent < self.BATTERY_LOW_THRESHOLD:
            if self.state not in [State.GO_TO_DOCK, State.CHARGING]:
                rospy.logwarn(f"Battery low ({self.battery_percent*100:.1f}%)! Going to dock.")
                self.stop_gvf_navigation()
                self.state = State.GO_TO_DOCK
                self.reset_light_seeking()
                self.start_gvf_navigation(self.DOCK_COORDS)

        # P2: Moisture low - override light-seeking (but not battery states)
        elif self.soil_moisture < self.MOISTURE_LOW_THRESHOLD:
            if self.state in [State.IDLE, State.LIGHT_SCAN, State.LIGHT_ALIGN, State.LIGHT_MOVE]:
                rospy.logwarn(f"Moisture low ({self.soil_moisture}%)! Going to water.")
                self.stop_gvf_navigation()
                self.state = State.GO_TO_WATER
                self.reset_light_seeking()
                self.start_gvf_navigation(self.WATER_COORDS)

        # Log current state
        rospy.loginfo_throttle(2, f"State: {self.state.value} | Nav: {self.nav_mode.value} | Battery: {self.battery_percent*100:.0f}% | Moisture: {self.soil_moisture}%")

        # === STATE MACHINE ===

        if self.state == State.IDLE:
            self.do_idle()

        elif self.state == State.GO_TO_DOCK:
            self.do_go_to_dock()

        elif self.state == State.CHARGING:
            self.do_charging()

        elif self.state == State.GO_TO_WATER:
            self.do_go_to_water()

        elif self.state == State.DOSING:
            self.do_dosing()

        elif self.state == State.LIGHT_SCAN:
            self.do_light_scan()

        elif self.state == State.LIGHT_ALIGN:
            self.do_light_align()

        elif self.state == State.LIGHT_MOVE:
            self.do_light_move()

    # === STATE HANDLERS ===

    def do_idle(self):
        self.stop()
        if self.is_daytime():
            rospy.loginfo("Daytime detected. Starting light-seeking.")
            self.state = State.LIGHT_SCAN
            self.reset_light_seeking()

    def do_go_to_dock(self):
        """GVF handles navigation, we just monitor arrival"""
        if self.check_arrival():
            rospy.loginfo("Arrived at dock. Charging...")
            self.stop_gvf_navigation()
            self.state = State.CHARGING

    def do_charging(self):
        self.stop()
        if self.battery_percent >= self.BATTERY_FULL:
            rospy.loginfo("Fully charged! Resuming behavior.")
            if self.is_daytime():
                self.state = State.LIGHT_SCAN
                self.reset_light_seeking()
            else:
                self.state = State.IDLE

    def do_go_to_water(self):
        """GVF handles navigation, we just monitor arrival"""
        if self.check_arrival():
            rospy.loginfo("Arrived at water station. Dosing...")
            self.stop_gvf_navigation()
            self.state = State.DOSING
            self.dose_start_time = rospy.Time.now()

    def do_dosing(self):
        self.stop()
        elapsed = (rospy.Time.now() - self.dose_start_time).to_sec()
        rospy.loginfo_throttle(1, f"Dosing... {elapsed:.1f}/{self.DOSE_DURATION}s")
        if elapsed >= self.DOSE_DURATION:
            rospy.loginfo("Dosing complete. Starting fresh light scan.")
            self.state = State.LIGHT_SCAN
            self.reset_light_seeking()

    def reset_light_seeking(self):
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_rotation = 0.0
        self.brightness_log = []
        self.move_start_pos = None
        self.bright_counter = 0

    def do_light_scan(self):
        """360° scan to find brightest direction - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        if self.image is None:
            rospy.logwarn_throttle(5, "Waiting for camera image...")
            return

        if self.scan_start_yaw is None:
            rospy.loginfo("Starting 360° light scan...")
            self.scan_start_yaw = self.current_yaw
            self.scan_last_yaw = self.current_yaw
            self.scan_accumulated_rotation = 0.0
            self.brightness_log = []

        brightness = self.get_brightness()

        # Log brightness at each angle
        if abs(self.angle_diff(self.current_yaw, self.scan_last_yaw)) > 0.01:
            self.brightness_log.append((self.current_yaw, brightness))

        # Track rotation
        delta = self.angle_diff(self.current_yaw, self.scan_last_yaw)
        self.scan_accumulated_rotation += abs(delta)
        self.scan_last_yaw = self.current_yaw

        if self.scan_accumulated_rotation < 2 * np.pi:
            # Keep rotating
            self.publish_direct_cmd(angular_z=0.4)
        else:
            # Scan complete
            self.stop()
            bright_angles = [b for _, b in self.brightness_log if b > self.BRIGHTNESS_SCAN_THRESHOLD]
            rospy.loginfo(f"Scan complete. Bright angles: {len(bright_angles)}/{len(self.brightness_log)}")

            if len(bright_angles) >= self.MIN_BRIGHT_ANGLES:
                rospy.loginfo("Environment is well-lit. Staying here.")
                # Wait a bit then rescan
                rospy.sleep(5.0)
                self.reset_light_seeking()
                return

            if self.brightness_log:
                self.target_yaw = max(self.brightness_log, key=lambda x: x[1])[0]
                rospy.loginfo(f"Brightest direction: {np.degrees(self.target_yaw):.1f}°")
                self.state = State.LIGHT_ALIGN
            else:
                rospy.logwarn("No brightness data. Rescanning.")
                self.reset_light_seeking()

    def do_light_align(self):
        """Align to brightest direction - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        error = self.angle_diff(self.target_yaw, self.current_yaw)

        if abs(error) > 0.05:
            self.publish_direct_cmd(angular_z=0.3 if error > 0 else -0.3)
        else:
            self.stop()
            rospy.loginfo("Aligned. Moving toward light.")
            if self.current_pose:
                self.move_start_pos = (self.current_pose[0], self.current_pose[1])
            self.bright_counter = 0
            self.state = State.LIGHT_MOVE

    def do_light_move(self):
        """Move toward light source - uses DIRECT control"""
        if self.nav_mode != NavigationMode.DIRECT:
            self.set_nav_mode(NavigationMode.DIRECT)

        if self.move_start_pos is None or self.current_pose is None:
            self.state = State.LIGHT_SCAN
            self.reset_light_seeking()
            return

        dist = np.hypot(self.current_pose[0] - self.move_start_pos[0],
                        self.current_pose[1] - self.move_start_pos[1])

        brightness = self.get_brightness()

        # Check if sustained brightness
        if brightness > self.BRIGHTNESS_MOVE_THRESHOLD:
            self.bright_counter += 1
            if self.bright_counter >= self.BRIGHT_CONFIRM_COUNT:
                rospy.loginfo("Found bright area. Rescanning.")
                self.stop()
                self.state = State.LIGHT_SCAN
                self.reset_light_seeking()
                return
        else:
            self.bright_counter = 0

        if dist < 1.0:
            self.publish_direct_cmd(linear_x=self.LIGHT_MOVE_SPEED)
        else:
            rospy.loginfo("Moved 1m. Rescanning.")
            self.stop()
            self.state = State.LIGHT_SCAN
            self.reset_light_seeking()


if __name__ == "__main__":
    BOTanicaBrain()
