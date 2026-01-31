#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Action Executor for BOTanica Agentic Controller

Executes robot actions (both high-level brain commands and low-level motor control)
and detects when they complete using sensor/pose feedback.
"""
import time
import math
import json
import numpy as np

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

from sensor_publisher.msg import SensorData
from distancefield.msg import Path as GVFPath


class ActionExecutor:
    """Executes actions and monitors completion."""

    # States the brain enters for each high-level command
    HIGH_LEVEL_STATES = {
        "GO_TO_WATER": ["GO_TO_WATER", "DOSING"],
        "GO_TO_DOCK": ["GO_TO_DOCK", "CHARGING"],
        "LIGHT_SCAN": ["LIGHT_SCAN", "LIGHT_ALIGN", "LIGHT_MOVE"],
    }

    def __init__(self, cmd_pub, voice_cmd_pub, nav_mode_pub, path_pub,
                 state_ref, default_move_speed=0.15, default_turn_speed=0.5,
                 arrival_tolerance=0.15):
        """
        Args:
            cmd_pub: Publisher for /cmd_vel_direct (Twist)
            voice_cmd_pub: Publisher for /voice_command (String)
            nav_mode_pub: Publisher for /nav_mode_gvf (Bool)
            path_pub: Publisher for /gvf/path (GVFPath)
            state_ref: Reference to agent node's shared state dict
                       (updated by callbacks in agent_node)
            default_move_speed: Default linear speed (m/s)
            default_turn_speed: Default angular speed (rad/s)
            arrival_tolerance: Distance tolerance for MOVE_TO (meters)
        """
        self.cmd_pub = cmd_pub
        self.voice_cmd_pub = voice_cmd_pub
        self.nav_mode_pub = nav_mode_pub
        self.path_pub = path_pub
        self.state = state_ref
        self.default_move_speed = default_move_speed
        self.default_turn_speed = default_turn_speed
        self.arrival_tolerance = arrival_tolerance

        # Abort flag - set by agent_node when STOP is received
        self.abort = False

    def execute(self, action: str, params: dict, timeout: float = 120.0) -> dict:
        """
        Execute an action and wait for completion.

        Args:
            action: Action name (e.g., "MOVE", "GO_TO_WATER")
            params: Action parameters
            timeout: Max seconds to wait for completion

        Returns:
            Result dict: {"success": bool, "message": str, "duration_s": float}
        """
        self.abort = False
        start_time = time.time()

        try:
            if action == "MOVE":
                return self._execute_move(params, timeout)
            elif action == "TURN":
                return self._execute_turn(params, timeout)
            elif action == "STRAFE":
                return self._execute_strafe(params, timeout)
            elif action == "WAIT":
                return self._execute_wait(params, timeout)
            elif action == "MOVE_TO":
                return self._execute_move_to(params, timeout)
            elif action in ("GO_TO_WATER", "GO_TO_DOCK", "LIGHT_SCAN", "STOP"):
                return self._execute_high_level(action, params, timeout)
            elif action == "READ_SENSORS":
                return self._execute_read_sensors()
            elif action == "SPEAK":
                msg = params.get("message", "")
                rospy.loginfo(f"[Agent]: {msg}")
                return {"success": True, "message": msg, "duration_s": 0.0}
            elif action == "DONE":
                summary = params.get("summary", "Goal complete")
                return {"success": True, "message": summary, "duration_s": 0.0}
            else:
                return {"success": False, "message": f"Unknown action: {action}", "duration_s": 0.0}

        except Exception as e:
            elapsed = time.time() - start_time
            rospy.logerr(f"Action {action} failed: {e}")
            self._stop_motors()
            return {"success": False, "message": str(e), "duration_s": elapsed}

    # === LOW-LEVEL MOVEMENT ===

    def _execute_move(self, params: dict, timeout: float) -> dict:
        """Drive forward/backward a specified distance."""
        distance = params.get("distance_m", 0.0)
        speed = min(abs(params.get("speed", self.default_move_speed)), 0.3)
        if distance < 0:
            speed = -speed

        self._set_nav_mode_direct()

        start_pos = self._get_position()
        if start_pos is None:
            return {"success": False, "message": "No position data available", "duration_s": 0.0}

        target_dist = abs(distance)
        start_time = time.time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self._stop_motors()
                return {"success": False, "message": f"MOVE timed out after {elapsed:.1f}s", "duration_s": elapsed}

            current_pos = self._get_position()
            if current_pos is None:
                self._publish_twist(linear_x=speed)
                rate.sleep()
                continue

            traveled = math.hypot(current_pos[0] - start_pos[0], current_pos[1] - start_pos[1])

            if traveled >= target_dist:
                self._stop_motors()
                return {"success": True, "message": f"Moved {traveled:.2f}m", "duration_s": elapsed}

            self._publish_twist(linear_x=speed)
            rate.sleep()

        self._stop_motors()
        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    def _execute_turn(self, params: dict, timeout: float) -> dict:
        """Rotate in place by specified degrees."""
        degrees = params.get("degrees", 0.0)
        speed_dps = min(abs(params.get("speed", 30.0)), 60.0)
        speed_rad = math.radians(speed_dps)
        if degrees < 0:
            speed_rad = -speed_rad

        self._set_nav_mode_direct()

        start_yaw = self.state.get("yaw_rad", 0.0)
        target_rad = abs(math.radians(degrees))
        accumulated = 0.0
        last_yaw = start_yaw
        start_time = time.time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self._stop_motors()
                return {"success": False, "message": f"TURN timed out after {elapsed:.1f}s", "duration_s": elapsed}

            current_yaw = self.state.get("yaw_rad", last_yaw)
            delta = self._angle_diff(current_yaw, last_yaw)
            accumulated += abs(delta)
            last_yaw = current_yaw

            if accumulated >= target_rad:
                self._stop_motors()
                actual_deg = math.degrees(accumulated)
                return {"success": True, "message": f"Turned {actual_deg:.1f} degrees", "duration_s": elapsed}

            self._publish_twist(angular_z=speed_rad)
            rate.sleep()

        self._stop_motors()
        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    def _execute_strafe(self, params: dict, timeout: float) -> dict:
        """Sideways movement using mecanum wheels."""
        distance = params.get("distance_m", 0.0)
        speed = min(abs(params.get("speed", self.default_move_speed)), 0.3)
        if distance < 0:
            speed = -speed

        self._set_nav_mode_direct()

        start_pos = self._get_position()
        if start_pos is None:
            return {"success": False, "message": "No position data available", "duration_s": 0.0}

        target_dist = abs(distance)
        start_time = time.time()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self._stop_motors()
                return {"success": False, "message": f"STRAFE timed out after {elapsed:.1f}s", "duration_s": elapsed}

            current_pos = self._get_position()
            if current_pos is None:
                self._publish_twist(linear_y=speed)
                rate.sleep()
                continue

            traveled = math.hypot(current_pos[0] - start_pos[0], current_pos[1] - start_pos[1])

            if traveled >= target_dist:
                self._stop_motors()
                return {"success": True, "message": f"Strafed {traveled:.2f}m", "duration_s": elapsed}

            self._publish_twist(linear_y=speed)
            rate.sleep()

        self._stop_motors()
        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    def _execute_wait(self, params: dict, timeout: float) -> dict:
        """Wait for a specified duration."""
        seconds = min(params.get("seconds", 1.0), timeout)
        start_time = time.time()
        rate = rospy.Rate(2)

        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed >= seconds:
                return {"success": True, "message": f"Waited {seconds:.1f}s", "duration_s": elapsed}
            rate.sleep()

        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    def _execute_move_to(self, params: dict, timeout: float) -> dict:
        """Navigate to absolute coordinates using GVF path planner."""
        target_x = params.get("x", 0.0)
        target_y = params.get("y", 0.0)

        current_pos = self._get_position()
        if current_pos is None:
            return {"success": False, "message": "No position data for MOVE_TO", "duration_s": 0.0}

        # Publish GVF path
        self._set_nav_mode_gvf()

        path_msg = GVFPath()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"

        start_point = Point32(x=current_pos[0], y=current_pos[1], z=0.0)
        end_point = Point32(x=target_x, y=target_y, z=0.0)

        path_msg.path.points = [start_point, end_point]
        path_msg.closed_path_flag = False
        path_msg.insert_n_points = 10
        path_msg.filter_path_n_average = 3

        self.path_pub.publish(path_msg)

        # Wait for arrival
        start_time = time.time()
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self._set_nav_mode_direct()
                self._stop_motors()
                return {"success": False, "message": f"MOVE_TO timed out after {elapsed:.1f}s", "duration_s": elapsed}

            current_pos = self._get_position()
            if current_pos is not None:
                dist = math.hypot(current_pos[0] - target_x, current_pos[1] - target_y)
                if dist < self.arrival_tolerance:
                    self._set_nav_mode_direct()
                    self._stop_motors()
                    return {"success": True, "message": f"Arrived at ({target_x:.2f}, {target_y:.2f})", "duration_s": elapsed}

            rate.sleep()

        self._set_nav_mode_direct()
        self._stop_motors()
        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    # === HIGH-LEVEL COMMANDS ===

    def _execute_high_level(self, action: str, params: dict, timeout: float) -> dict:
        """Execute a high-level command by publishing to /voice_command."""
        # Publish in the format the brain expects: "COMMAND|text|force|source"
        msg_data = f"{action}|agent command|False|agent"
        self.voice_cmd_pub.publish(String(data=msg_data))

        if action == "STOP":
            return {"success": True, "message": "Stop command sent", "duration_s": 0.0}

        # Wait for the brain to finish processing the command
        expected_states = self.HIGH_LEVEL_STATES.get(action, [])
        if not expected_states:
            return {"success": True, "message": f"{action} sent", "duration_s": 0.0}

        start_time = time.time()
        rate = rospy.Rate(2)

        # Phase 1: Wait for brain to enter the expected state
        entered = False
        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                return {"success": False, "message": f"{action} timed out waiting to start", "duration_s": elapsed}

            brain_state = self.state.get("brain_state", "IDLE")
            if brain_state in expected_states:
                entered = True
                break
            if elapsed > 5.0:
                # Give up waiting for state entry after 5s - maybe it transitioned fast
                entered = True
                break
            rate.sleep()

        # Phase 2: Wait for brain to leave the expected state (action complete)
        while not rospy.is_shutdown() and not self.abort:
            elapsed = time.time() - start_time
            if elapsed > timeout:
                return {"success": False, "message": f"{action} timed out during execution", "duration_s": elapsed}

            brain_state = self.state.get("brain_state", "IDLE")
            if brain_state not in expected_states:
                return {"success": True, "message": f"{action} completed (state: {brain_state})", "duration_s": elapsed}

            rate.sleep()

        return {"success": False, "message": "Aborted", "duration_s": time.time() - start_time}

    # === META ACTIONS ===

    def _execute_read_sensors(self) -> dict:
        """Return current sensor state."""
        return {
            "success": True,
            "message": "Sensor data read",
            "duration_s": 0.0,
            "sensors": {
                "battery_pct": self.state.get("battery_pct", -1),
                "moisture_pct": self.state.get("moisture_pct", -1),
                "sunlight_lux": self.state.get("sunlight_lux", -1),
                "temperature_c": self.state.get("temperature_c", -1),
            },
        }

    # === HELPERS ===

    def _get_position(self):
        """Get current (x, y) position or None."""
        x = self.state.get("pos_x")
        y = self.state.get("pos_y")
        if x is not None and y is not None:
            return (x, y)
        return None

    def _angle_diff(self, a, b):
        """Compute shortest angular difference between two angles in radians."""
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _publish_twist(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """Publish a Twist command to /cmd_vel_direct."""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    def _stop_motors(self):
        """Publish zero velocity."""
        self._publish_twist(0, 0, 0)

    def _set_nav_mode_direct(self):
        """Switch mux to direct control."""
        msg = Bool()
        msg.data = False
        self.nav_mode_pub.publish(msg)

    def _set_nav_mode_gvf(self):
        """Switch mux to GVF control."""
        msg = Bool()
        msg.data = True
        self.nav_mode_pub.publish(msg)
