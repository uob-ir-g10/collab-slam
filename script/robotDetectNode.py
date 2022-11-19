#!/usr/bin/python3

"""
This node listens to:
    * scans published to `/robot_i/base_scan`
    * base truth poses from `/robot_i/base_pose_ground_truth`
    * a ground truth map from `map` 
And publishes the poses of detected robots to `/robot_i/detected_robots`, 
that are in the laser scan range everytime robot_i executes a scan
"""

import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import math
import sys

class RobotDetectionNode(object):
    def __init__(self, num_robots):
        self.robot_poses = []
        self._detected_robot_publishers = []
        for i in range(num_robots):
            self._detected_robot_publishers.append(rospy.Publisher(f"/robot_{i}/detected_robots", PoseArray))
            # usnure about queue_size=1
            self._laser_subscriber = rospy.Subscriber(
                f"/robot_{i}/base_scan", LaserScan, self._laser_callback, i, queue_size=1)
            self._ground_truth_poses_subscriber = rospy.Subscriber(
                f"/robot_{i}/base_pose_ground_truth", Odometry, self._ground_truth_pose_callback, i)
            self.robot_poses.append(None)

        rospy.loginfo("Waiting for a map... (run `rosrun map_server map_server <mapname>`)")
        try:
            occupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (occupancy_map.info.width, occupancy_map.info.height,
                       occupancy_map.info.resolution))
        self.ROWS = occupancy_map.info.height
        self.COLUMNS = occupancy_map.info.width
        self.RESOLUTION = occupancy_map.info.resolution

    def _ground_truth_pose_callback(self, odom, i):
        pos = odom.pose.pose
        self.robot_poses[i] = pos
    
    def _laser_callback(self, scan, robot_num):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = scan
        self_pos = self.robot_poses[robot_num]
        detected_bots = PoseArray()

        for other in self.robot_poses:
            if other == self_pos:
                continue
            vectorTo = (other.position.x - self_pos.position.x, other.position.y - self_pos.position.y)

            # convert to polar
            globalAngleTo = math.atan2(vectorTo[1], vectorTo[0])
            robotHeading = getHeading(self_pos.orientation)
            
            # angle to other robot, relative to own heading
            angleTo = (globalAngleTo - robotHeading + math.pi) % (2 * math.pi) - math.pi

            # check if other robot is within scan angle        
            if angleTo > scan.angle_max or angleTo < scan.angle_min:
                continue

            # check if there are no walls in between
            distanceTo = math.sqrt(vectorTo[0]**2 + vectorTo[1]**2)
            scanDistance = scan.ranges[int((angleTo - scan.angle_min) // scan.angle_increment)]
            if distanceTo < scanDistance and distanceTo < scan.range_max:
                detected_bots.poses.append(other)
        
        # publish detected bots to right topic
        rospy.loginfo(detected_bots)
        self._detected_robot_publishers[robot_num].publish(detected_bots)

def getHeading(q):
    """
    Get the robot heading in radians from a Quaternion representation.
    
    :Args:
        | q (geometry_msgs.msg.Quaternion): a orientation about the z-axis
    :Return:
        | (double): Equivalent orientation about the z-axis in radians
    """
    yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                     q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw


if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("robot_detector")
    node = RobotDetectionNode(2)
    rospy.spin()