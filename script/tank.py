#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from socspioneer.msg import Shot

class Tank(object):
    def __init__(self, namespace, can_fire = True):
        self.namespace = namespace
        self.can_fire = can_fire
        self.health = 100

        # Continuously update own pose based on published ground_truth information
        self.pose = None
        self._ground_truth_poses_subscriber = rospy.Subscriber(
                f"/{namespace}/base_pose_ground_truth", Odometry, self._ground_truth_pose_callback)

        self.shots_fired_publisher = rospy.Publisher(f"{namespace}/shots_fired", Shot, queue_size=0)
    
    def _ground_truth_pose_callback(self, odom):
        self.pose = odom.pose.pose

    def fire(self, target: Point):
        origin = Point(**self.pose.position)
        self.shots_fired_publisher.publish(Shot(origin, target))
        rospy.loginfo("shot fired")

