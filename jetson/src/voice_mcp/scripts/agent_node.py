#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BOTanica Agent Node - LLM-powered agentic robot controller

Sits on top of botanica_brain.py as a higher-level controller.
Receives natural language goals via /voice_raw, uses GPT-4 to decompose
them into sequences of primitive actions, and executes them in a loop.

Simple commands (stop, go to water, etc.) are passed through directly
without invoking the LLM.

Architecture:
    Voice -> Whisper -> /voice_raw -> [this node] -> /voice_command (high-level)
                                                  -> /cmd_vel_direct (low-level)
"""
import json
import math
import threading

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion

from sensor_publisher.msg import SensorData
from distancefield.msg import Path as GVFPath

from command_parser import CommandParser
from gpt_planner import GPTPlanner
from action_executor import ActionExecutor


class AgentNode:
    """ROS node for LLM-powered agentic robot control."""

    def __init__(self):
        rospy.init_node("agent_controller")

        # Parameters
        self.gpt_model = rospy.get_param("~gpt_model", "gpt-4")
        self.max_steps = rospy.get_param("~max_steps", 20)
        self.step_timeout = rospy.get_param("~step_timeout", 120.0)
        self.move_speed = rospy.get_param("~move_speed", 0.15)
        self.turn_speed = rospy.get_param("~turn_speed", 0.5)

        # Shared state dict (updated by callbacks, read by executor)
        self.state = {
            "battery_pct": 100.0,
            "moisture_pct": 100,
            "sunlight_lux": 0,
            "temperature_c": 0.0,
            "fertility": 0,
            "pos_x": None,
            "pos_y": None,
            "yaw_rad": 0.0,
            "brain_state": "IDLE",
            "force_override": False,
        }

        # Command parser for simple pass-through
        self.parser = CommandParser()

        # GPT planner
        self.planner = GPTPlanner(model=self.gpt_model)

        # === PUBLISHERS ===
        self.cmd_pub = rospy.Publisher("/cmd_vel_direct", Twist, queue_size=10)
        self.voice_cmd_pub = rospy.Publisher("/voice_command", String, queue_size=10)
        self.nav_mode_pub = rospy.Publisher("/nav_mode_gvf", Bool, queue_size=1)
        self.path_pub = rospy.Publisher("/gvf/path", GVFPath, queue_size=1)
        self.agent_status_pub = rospy.Publisher("/agent/status", String, queue_size=1)

        # Action executor
        self.executor = ActionExecutor(
            cmd_pub=self.cmd_pub,
            voice_cmd_pub=self.voice_cmd_pub,
            nav_mode_pub=self.nav_mode_pub,
            path_pub=self.path_pub,
            state_ref=self.state,
            default_move_speed=self.move_speed,
            default_turn_speed=self.turn_speed,
        )

        # === SUBSCRIBERS ===
        rospy.Subscriber("/voice_raw", String, self.voice_raw_callback)
        rospy.Subscriber("/battery", BatteryState, self.battery_callback)
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/botanica/state", String, self.brain_state_callback)

        # Agent execution state
        self.goal_active = False
        self.goal_thread = None
        self.goal_lock = threading.Lock()

        rospy.loginfo("Agent Controller initialized")
        rospy.loginfo(f"  GPT model: {self.gpt_model}")
        rospy.loginfo(f"  Max steps: {self.max_steps}")
        rospy.loginfo(f"  Step timeout: {self.step_timeout}s")

    # === CALLBACKS ===

    def battery_callback(self, msg):
        self.state["battery_pct"] = round(msg.percentage * 100, 1)

    def sensor_callback(self, msg):
        self.state["moisture_pct"] = msg.moisture
        self.state["sunlight_lux"] = msg.sunlight
        self.state["temperature_c"] = round(msg.temperature, 1)
        self.state["fertility"] = msg.fertility

    def pose_callback(self, msg):
        pos = msg.pose.position
        orient = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.state["pos_x"] = round(pos.x, 3)
        self.state["pos_y"] = round(pos.y, 3)
        self.state["yaw_rad"] = yaw

    def odom_callback(self, msg):
        orient = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        # Only use odom if no OptiTrack pose
        if self.state["pos_x"] is None:
            pos = msg.pose.pose.position
            self.state["pos_x"] = round(pos.x, 3)
            self.state["pos_y"] = round(pos.y, 3)
        self.state["yaw_rad"] = yaw

    def brain_state_callback(self, msg):
        self.state["brain_state"] = msg.data

    def voice_raw_callback(self, msg):
        """
        Handle raw voice transcription.
        Simple commands -> pass through directly.
        Complex goals -> start agentic loop.
        """
        text = msg.data.strip()
        if not text:
            return

        rospy.loginfo(f"Agent received: '{text}'")

        # Check for simple pass-through commands
        parsed = self.parser.parse(text)
        if parsed["recognized"]:
            cmd = parsed["command"]

            # STOP always goes through immediately and aborts any active goal
            if cmd == "STOP":
                rospy.loginfo(f"Pass-through: {cmd}")
                self._abort_goal()
                msg_data = f"{cmd}|{text}|{parsed['force']}|voice"
                self.voice_cmd_pub.publish(String(data=msg_data))
                self._publish_status("idle", "Stop command executed")
                return

            # If no goal is active, simple commands pass through without LLM
            if not self.goal_active:
                rospy.loginfo(f"Pass-through: {cmd} (force={parsed['force']})")
                msg_data = f"{cmd}|{text}|{parsed['force']}|voice"
                self.voice_cmd_pub.publish(String(data=msg_data))
                self._publish_status("pass_through", f"Executed {cmd}")
                return

        # Complex goal or unrecognized command -> start agentic loop
        # If a goal is already active, abort it first
        if self.goal_active:
            rospy.loginfo("New goal received, aborting current goal")
            self._abort_goal()

        # Start new goal in background thread
        self.goal_thread = threading.Thread(target=self._execute_goal, args=(text,))
        self.goal_thread.daemon = True
        self.goal_thread.start()

    # === AGENTIC LOOP ===

    def _execute_goal(self, user_goal: str):
        """Main agentic loop - runs in a background thread."""
        with self.goal_lock:
            self.goal_active = True
            self.executor.abort = False

        rospy.loginfo(f"=== Starting agentic goal: '{user_goal}' ===")
        self._publish_status("planning", f"Goal: {user_goal}")

        # Build initial conversation
        messages = [GPTPlanner.build_system_message()]
        state_snapshot = self._get_state_snapshot()
        messages.append(GPTPlanner.build_user_message(user_goal, state_snapshot))

        for step in range(1, self.max_steps + 1):
            if rospy.is_shutdown() or self.executor.abort:
                rospy.loginfo("Goal aborted")
                self._publish_status("aborted", "Goal was aborted")
                break

            rospy.loginfo(f"--- Step {step}/{self.max_steps} ---")
            self._publish_status("thinking", f"Step {step}: Planning next action...")

            # Ask GPT-4 for next action
            action = self.planner.get_next_action(messages)
            messages.append(GPTPlanner.build_assistant_message(action))

            action_name = action.get("action", "UNKNOWN")
            params = action.get("params", {})
            reasoning = action.get("reasoning", "")

            rospy.loginfo(f"GPT-4 action: {action_name} params={params}")
            rospy.loginfo(f"  Reasoning: {reasoning}")

            # Check for DONE
            if action_name == "DONE":
                summary = params.get("summary", "Goal complete")
                rospy.loginfo(f"=== Goal complete: {summary} ===")
                self._publish_status("done", summary)
                break

            # Check for SPEAK (no execution needed, just log)
            if action_name == "SPEAK":
                msg = params.get("message", "")
                rospy.loginfo(f"[Agent says]: {msg}")
                self._publish_status("speaking", msg)
                # Add observation and continue
                state_snapshot = self._get_state_snapshot()
                result = {"success": True, "message": f"Spoke: {msg}", "duration_s": 0.0}
                messages.append(GPTPlanner.build_observation_message(result, state_snapshot))
                continue

            # Execute the action
            self._publish_status("executing", f"Step {step}: {action_name}")
            result = self.executor.execute(action_name, params, timeout=self.step_timeout)

            rospy.loginfo(f"Result: {result}")

            # Get updated state
            state_snapshot = self._get_state_snapshot()
            messages.append(GPTPlanner.build_observation_message(result, state_snapshot))

            # If action failed, GPT-4 can decide to retry or adjust
            if not result.get("success", False):
                rospy.logwarn(f"Action {action_name} failed: {result.get('message', '')}")

        else:
            rospy.logwarn(f"Max steps ({self.max_steps}) reached. Goal may be incomplete.")
            self._publish_status("max_steps", "Reached maximum step limit")

        with self.goal_lock:
            self.goal_active = False

    def _abort_goal(self):
        """Abort any active goal."""
        self.executor.abort = True
        if self.goal_thread and self.goal_thread.is_alive():
            self.goal_thread.join(timeout=3.0)
        with self.goal_lock:
            self.goal_active = False

    # === STATE ===

    def _get_state_snapshot(self) -> dict:
        """Build a state snapshot for the LLM."""
        yaw_deg = round(math.degrees(self.state.get("yaw_rad", 0.0)), 1)
        pos_x = self.state.get("pos_x")
        pos_y = self.state.get("pos_y")

        return {
            "battery_pct": self.state.get("battery_pct", -1),
            "moisture_pct": self.state.get("moisture_pct", -1),
            "sunlight_lux": self.state.get("sunlight_lux", -1),
            "temperature_c": self.state.get("temperature_c", -1),
            "position": {
                "x": pos_x if pos_x is not None else "unknown",
                "y": pos_y if pos_y is not None else "unknown",
                "yaw_deg": yaw_deg,
            },
            "brain_state": self.state.get("brain_state", "UNKNOWN"),
            "force_override": self.state.get("force_override", False),
        }

    def _publish_status(self, status: str, message: str):
        """Publish agent status for monitoring."""
        data = json.dumps({"status": status, "message": message})
        self.agent_status_pub.publish(String(data=data))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = AgentNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
