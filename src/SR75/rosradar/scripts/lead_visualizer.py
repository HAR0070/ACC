#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Bool, Int32
from visualization_msgs.msg import Marker

class LeadVisualizer:

    def __init__(self):
        rospy.init_node("lead_visualizer")

        self.d_lead = None
        self.v_rel = 0.0
        self.valid = False
        self.rcs = 0

        rospy.Subscriber("/lead_distance", Float32, self.dist_cb)
        rospy.Subscriber("/lead_relative_velocity", Float32, self.vel_cb)
        rospy.Subscriber("/lead_valid", Bool, self.valid_cb)
        rospy.Subscriber("/lead_intensity", Int32, self.rcs_cb)

        self.marker_pub = rospy.Publisher(
            "/lead_marker", Marker, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_marker)

        rospy.loginfo("Lead Visualizer Started")

    def dist_cb(self, msg):
        self.d_lead = msg.data

    def vel_cb(self, msg):
        self.v_rel = msg.data

    def valid_cb(self, msg):
        self.valid = msg.data

    def rcs_cb(self, msg):
        self.rcs = msg.data

    def publish_marker(self, event):

        marker = Marker()
        marker.header.frame_id = "os_sensor_right"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lead_object"
        marker.id = 0
        marker.type = Marker.CUBE

        if not self.valid or self.d_lead is None:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = self.d_lead
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.75
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 3.0

        # Color (change based on distance maybe later)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)


if __name__ == "__main__":
    try:
        LeadVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
