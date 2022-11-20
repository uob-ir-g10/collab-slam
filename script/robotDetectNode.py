#!/usr/bin/python3

"""
This node listens to:
    * scans published to `/robot_i/base_scan`
    * base truth poses from `/robot_i/base_pose_ground_truth`
And publishes the poses of detected robots to `/robot_i/detected_robots`, 
that are in the laser scan range everytime robot_i executes a scan
"""

import rospy
from geometry_msgs.msg import PoseArray, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import tf2_ros
import tf_conversions

class RobotDetectionNode(object):
    def __init__(self, num_robots):
        self.tf2_br = tf2_ros.TransformBroadcaster()

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


    def _ground_truth_pose_callback(self, odom, i):
        pos = odom.pose.pose
        self.robot_poses[i] = pos
    
    def _laser_callback(self, scan, robot_num):
        """
        Laser received. Check if any other robots are within this robot's view cone,
        and publish it to the corresponding topic.
        """
        self_pos = self.robot_poses[robot_num]
        transforms = []
        detected_bots = PoseArray()

        for i, other in enumerate(self.robot_poses):
            if i == robot_num:
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

                # broadcast a transform from this robot's base_link toward the other robot it sees
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = f"robot_{robot_num}/base_link"
                t.child_frame_id = f"robot_{i}"
                t.transform.translation.x = distanceTo * math.cos(angleTo) 
                t.transform.translation.y = distanceTo * math.sin(angleTo) 
                t.transform.translation.z = 0.0
                q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                transforms.append(t)

        # publish detected bots to right topic
        self._detected_robot_publishers[robot_num].publish(detected_bots)

        # broadcast transforms
        self.tf2_br.sendTransform(transforms)

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