#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid


def callback(data):
    rospy.loginfo("{}x{} map received at {}, origin:\n {}"
                  .format(data.info.width, data.info.height, data.header.stamp, data.info.origin))
    occupancy_grid = data.data
    first = next(e for e, i in enumerate(occupancy_grid) if i != -1)
    row = int(first / 4000) * 0.05000000074505806 - 100
    column = (first % 4000) * 0.05000000074505806 - 100
    rospy.loginfo("First known cell: {} {}".format(row, column))
    #segment = occupancy_grid[:400]
    #dim = 20
    #for i in range(dim):
    #    rospy.loginfo(segment[dim*i:dim*(i+1)])
    # rospy.loginfo("first non -1 item is: {}".format(first))

    # rospy.loginfo("Current Grid: {} nrs".format(len(occupancy_grid)))


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("robot_0/map", OccupancyGrid, callback)

    rospy.spin()

if __name__=="__main__":
    listener()