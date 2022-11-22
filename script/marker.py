#!/usr/bin/python3

"""
Node showing how to publish markers to rviz
"""

import rospy
from visualization_msgs.msg import Marker

class MarkerNode(object):
    def __init__(self):
        pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)
        marker = Marker()

        # Frame ID on which the marker will be placed (see TF)
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        
        # Namespace and ID for marker. Any marker sent with same namespace and id will overwrite old one
        marker.ns = "my_marker"
        marker.id = 0

        marker.type = Marker.CUBE

        # Marker action: ADD DELETE DELETEALL
        marker.action = Marker.ADD

        # Marker pose
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Marker scale (in meters)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime = rospy.Duration()
        
        # Need to sleep a bit before publishing to the topic
        rospy.sleep(0.5)
        pub.publish(marker)


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("marker")
    node = MarkerNode()
    #rospy.spin()