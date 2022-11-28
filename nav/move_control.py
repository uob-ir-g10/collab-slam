#!/usr/bin/python3

import rospy
import tf2_ros
from motion_controller import motion_controller
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Log

pub = rospy.Publisher("robot_0/cmd_vel", Twist, queue_size=10)
logger = rospy.Publisher("rosout", Log, queue_size=10) 

def listen_maps():

    rospy.init_node("listen_maps", anonymous=True)
    rospy.Subscriber = ("map", OccupancyGrid, callback)
    rospy.spin()


def callback(current_map):
    rospy.logwarn("callback happened")
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        trans = tf_buffer.lookup_transform("map","odom",rospy.Time())
    except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass

    # velocity = joe.get_command(trans, current_map.data)
    
    

    move_cmd = Twist()
    log = Log()
    log.name = "test"
    log.msg=trans
    logger.publish(log)
    # move_cmd.linear.x = velocity[0]
    # move_cmd.angular.z = velocity[1]
    pub.publish(move_cmd)


if __name__ == "__main__":
    try:
        listen_maps()
    except(rospy.ROSInterruptException):
        pass

