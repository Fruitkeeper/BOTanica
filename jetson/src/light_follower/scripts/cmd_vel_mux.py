#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
cmd_vel Multiplexer

Switches between two cmd_vel sources based on navigation mode:
- /cmd_vel_direct: Direct control from botanica_brain (light-seeking)
- /cmd_vel_gvf: GVF vectorfield_stack output (waypoint navigation)

Outputs to /cmd_vel which goes to the RoboMaster driver on the Pi.
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelMux:
    def __init__(self):
        rospy.init_node("cmd_vel_mux")

        # Current mode: True = GVF, False = Direct
        self.use_gvf = False

        # Last received commands
        self.cmd_direct = Twist()
        self.cmd_gvf = Twist()

        # Timestamps for timeout
        self.last_direct_time = rospy.Time.now()
        self.last_gvf_time = rospy.Time.now()
        self.cmd_timeout = rospy.Duration(0.5)  # 500ms timeout

        # Publisher - final cmd_vel to robot
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("/cmd_vel_direct", Twist, self.direct_callback)
        rospy.Subscriber("/cmd_vel_gvf", Twist, self.gvf_callback)
        rospy.Subscriber("/nav_mode_gvf", Bool, self.mode_callback)

        rospy.loginfo("cmd_vel_mux initialized")
        rospy.loginfo("  Direct topic: /cmd_vel_direct")
        rospy.loginfo("  GVF topic: /cmd_vel_gvf")
        rospy.loginfo("  Output topic: /cmd_vel")
        rospy.loginfo("  Mode topic: /nav_mode_gvf")

        # Publish at 20Hz
        rospy.Timer(rospy.Duration(0.05), self.publish_cmd)
        rospy.spin()

    def direct_callback(self, msg):
        self.cmd_direct = msg
        self.last_direct_time = rospy.Time.now()

    def gvf_callback(self, msg):
        self.cmd_gvf = msg
        self.last_gvf_time = rospy.Time.now()

    def mode_callback(self, msg):
        new_mode = msg.data
        if new_mode != self.use_gvf:
            self.use_gvf = new_mode
            mode_str = "GVF" if self.use_gvf else "DIRECT"
            rospy.loginfo(f"cmd_vel_mux: Switched to {mode_str} mode")

    def publish_cmd(self, _):
        now = rospy.Time.now()
        cmd = Twist()

        if self.use_gvf:
            # Use GVF command if recent
            if (now - self.last_gvf_time) < self.cmd_timeout:
                cmd = self.cmd_gvf
            else:
                rospy.logwarn_throttle(2, "GVF cmd_vel timeout - stopping")
        else:
            # Use direct command if recent
            if (now - self.last_direct_time) < self.cmd_timeout:
                cmd = self.cmd_direct
            else:
                rospy.logwarn_throttle(2, "Direct cmd_vel timeout - stopping")

        self.cmd_pub.publish(cmd)


if __name__ == "__main__":
    CmdVelMux()
