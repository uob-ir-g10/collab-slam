#!/usr/bin/python3

"""
This node listens to:
    * scans published to `/robot_i/base_scan`
    * base truth poses from `/robot_i/base_pose_ground_truth`
And publishes the poses of detected robots to `/robot_i/detected_robots`, 
that are in the laser scan range everytime robot_i executes a scan
"""

import rospy
from typing import List
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseArray, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from socspioneer.msg import Shot
import math
import tf2_ros
import tf_conversions
from visualization_msgs.msg import Marker

# add cooldonw to firing
# add duration limit to rviz marker

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
        self.marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=0)

    
    def _ground_truth_pose_callback(self, odom):
        self.pose = odom.pose.pose


    def fire(self, target: Point):
        if not self.can_fire:
            rospy.logerr(f"Tank {self.namespace} tried firing, but it doesn't have the capability to")
            return
        origin = self.pose.position
        self.shots_fired_publisher.publish(Shot(origin, target))
        marker = Marker()

        # Frame ID on which the marker will be placed (see TF)
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time()
        
        # Namespace and ID for marker. Any marker sent with same namespace and id will overwrite old one
        marker.ns = "my_marker"
        marker.id = 0

        marker.type = Marker.LINE_STRIP

        # Marker action: ADD DELETE DELETEALL
        marker.action = Marker.ADD

        # Marker pose
        marker.pose.position = Point(0, 0, 0)
        marker.pose.orientation = Quaternion(0, 0, 0, 1)

        # Line length
        marker.scale.x = 0.03

        marker.points = []
        marker.points.append(origin)
        marker.points.append(target)

        # Color
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

        marker.lifetime = rospy.Duration()
        
        self.marker_publisher.publish(marker)
        rospy.loginfo("pew")



class GameManager(object):
    def __init__(self, num_robots, detector_tanks=None):
        tanks = [Tank(f"robot_{i}") for i in range(num_robots)]
        self.tf2_br = tf2_ros.TransformBroadcaster()

        if detector_tanks is None:
            detector_tanks = tanks

        self.all_tanks = tanks
        self._detected_robot_publishers = {}
        for tank in detector_tanks:
            tank_ns = tank.namespace
            self._detected_robot_publishers[tank] = rospy.Publisher(f"/{tank_ns}/detected_robots", PoseArray, queue_size=0)
            # unsure about queue_size=1
            self._laser_subscriber = rospy.Subscriber(
                f"/{tank_ns}/base_scan", LaserScan, self._laser_callback, tank, queue_size=1)

    
    def _laser_callback(self, scan, scan_tank):
        """
        Laser received. Check if any other robots are within this robot's view cone,
        and publish it to the corresponding topic.
        """
        if scan_tank.pose == None:
            return 
        transforms = []
        detected_bots = PoseArray()

        for tank in self.all_tanks:
            if tank.namespace == scan_tank.namespace or tank.pose == None:
                continue
            vectorTo = (tank.pose.position.x - scan_tank.pose.position.x, tank.pose.position.y - scan_tank.pose.position.y)

            # convert to polar
            globalAngleTo = math.atan2(vectorTo[1], vectorTo[0])
            robotHeading = getHeading(scan_tank.pose.orientation)
            
            # angle to other robot, relative to own heading
            angleTo = (globalAngleTo - robotHeading + math.pi) % (2 * math.pi) - math.pi

            # check if other robot is within scan angle        
            if angleTo > scan.angle_max or angleTo < scan.angle_min:
                continue

            # check if there are no walls in between
            distanceTo = math.sqrt(vectorTo[0]**2 + vectorTo[1]**2)
            scanDistance = scan.ranges[int((angleTo - scan.angle_min) // scan.angle_increment)]
            if distanceTo < scanDistance and distanceTo < scan.range_max:
                detected_bots.poses.append(tank.pose)
                scan_tank.fire(tank.pose.position)

                # broadcast a transform from this robot's base_link toward the other robot it sees
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = f"{scan_tank.namespace}/base_link"
                t.child_frame_id = f"{tank.namespace}"
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
        self._detected_robot_publishers[scan_tank].publish(detected_bots)


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
    rospy.init_node("game_manager")
    node = GameManager(2)
    rospy.spin()