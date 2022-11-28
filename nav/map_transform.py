#!/usr/bin/env python3

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('map_transform')
    rospy.loginfo("node initialized")
    rate = rospy.Rate(100)
    while True:
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(), "map", "robot_0/base_link")
        rate.sleep()
