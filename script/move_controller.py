#!/usr/bin/python3
import random

import rospy
import traceback
import math
from astar.astar import *
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class Controller:
    def __init__(self, namespace):
        self.goals = [[1,3], [-2, 2.5], [1,3], [0,4], [0, -2], [-1, -2.5], [-4, -3]]
        self.current_goal = 0
        self.namespace = namespace
        self.cmd_vel_publisher = rospy.Publisher(f"{namespace}/cmd_vel", Twist, queue_size=1)

        self.pose = None
        self._ground_truth_poses_subscriber = rospy.Subscriber(
            f"/{namespace}/base_pose_ground_truth", Odometry, self._ground_truth_pose_callback)
        self.move_cycles = 0

        self.mapinfo = rospy.wait_for_message(f"{namespace}/map", OccupancyGrid).info
        self.grid_res = self.mapinfo.resolution
        self.grid_dim = [self.mapinfo.width, self.mapinfo.height]
        self.grid_origin = [self.mapinfo.origin.position.x, self.mapinfo.origin.position.y]

        self.last_scan = None
        self._base_scan_subscriber = rospy.Subscriber(
            f"/{namespace}/base_scan", LaserScan, self._base_scan_callback, queue_size=1)
        rospy.logdebug("Waiting for map...")
        rospy.sleep(3)

        self.tried_backward = False
        self.immobile = False

        self.run()

    def run(self):
        rospy.loginfo("running started")
        op = 0
        last = 0
        while True:
            if op == 0:
                if last == 0:
                    rospy.loginfo("Navigating...")
                    goal = self.get_new_goal()
                    last = 1
                else:
                    rospy.loginfo("Exploring local area...")
                    self.explore()
                    last = 0

            for i in range(5):
                op = self.execute_movement(goal)
                if op != -1:
                    last = 1
                    break
                rospy.logwarn(f"Couldn't find path to goal, trying again...code: {op}")
                op = 0
            if i == 4:
                rospy.logwarn("Couldn't find path, Exploring...")
                last = 1

    def get_new_goal(self):
        goal = Twist()
        goal.linear.x = 1
        goal.linear.y = 3
        return goal
        grid = rospy.wait_for_message(f"/{self.namespace}/map", OccupancyGrid).data

        coord_2D = get_grid_coord(self.pose.position, self.grid_origin, self.grid_res)
        coord_1D = get_1D(*coord_2D, self.grid_dim[0])

        unknowns = check_neighbours(Node(coord_1D, None, 0, 0, 0, self.grid_dim[0]), grid, self.grid_dim[0], 4, -1)
        random_choice = unknowns[random.randint(0, len(unknowns))]

        choice_grid_coords = get_2D(random_choice, self.grid_dim[0])

        goal.linear.x = get_world_coord(choice_grid_coords)[0]
        goal.linear.y = get_world_coord(choice_grid_coords)[1]
        #goal.linear.x = self.goals[self.current_goal][0]
        #goal.linear.y = self.goals[self.current_goal][1]
        #self.current_goal += 1
        return goal

    def execute_movement(self, goal):
        tank_pose = self.pose.position

        grid_coord_i, grid_coord_j = get_grid_coord(tank_pose, self.grid_origin, self.grid_res)
        grid_coord_1D = round(get_1D(grid_coord_i, grid_coord_j, self.grid_dim[0]))

        # get goal
        goal = get_grid_coord(goal.linear, self.grid_origin, self.grid_res)
        goal_1D = round(get_1D(*goal, self.grid_dim[0]))
        grid_1D = rospy.wait_for_message(f"/{self.namespace}/map", OccupancyGrid).data

        goal = find_path(grid_1D, grid_coord_1D, goal_1D, self.grid_dim[0], 4)
        if not goal:
            goal = find_path(grid_1D, grid_coord_1D, goal_1D, self.grid_dim[0], 3)
            if not goal:
                return -1
        # translate to waypoints
        vectors_2D = translate_path(traverse_path(goal, []), self.grid_dim[0])
        real_vectors = [get_world_coord(vector, self.grid_res) for vector in vectors_2D]

        return self.move_to_goal(real_vectors)


    def move_to_goal(self, waypoints):
        previous = [self.pose.position.x, self.pose.position.y]
        for e, point in enumerate(waypoints):
            waypoints[e][0] = previous[0] + point[0]
            waypoints[e][1] = previous[1] + point[1]
            previous = point
        #rospy.loginfo(waypoints)
        for e, point in enumerate(waypoints):
            starting_point = self.pose.position
            x = point[0] - starting_point.x
            y = point[1] - starting_point.y

            if x != 0:
                angle = math.atan2(y, x)
            else:
                if y <0:
                    angle = -(math.pi / 2)
                else:
                    angle = (math.pi / 2)
            #rospy.loginfo(f"{angle} is what we are looking for at{x} {y}")
            if angle < getHeading(self.pose.orientation):
                rotation_dir = -1
            else:
                rotation_dir = 1
            while True:
                diff = angle - getHeading(self.pose.orientation)
                if 0.015 >= diff >= -0.015:
                    #rospy.loginfo(f"stopped rotating at: \n{getHeading(self.pose.orientation)} at ({x} {y})")
                    break
                rotate = Twist()
                rotate.angular.z = abs(diff) * rotation_dir
                self.move(rotate)

            while True:
                diff_x = abs(self.pose.position.x - point[0])
                diff_y = abs(self.pose.position.y - point[1])

                if diff_y <= 0.03 and diff_x < 0.03:
                    break
                forwards = Twist()
                forwards.linear.x = (diff_x + diff_y)/2
                self.move(forwards)
        return 0

    def explore(self):
        start = rospy.Time.now()
        delta = rospy.Duration(10)
        self.tried_backward = False
        while True:
            move_cmd = Twist()
            # move_cmd.angular.x = 1
            elapsed = rospy.Time.now() - start
            if elapsed > delta:
                break
            if self.move_cycles > 0:
                self.move_cycles -= 1
            elif self.move_cycles == 0:
                self.move_cycles -= 1
                self.immobile = False

            distance_left = self.get_scan_distance(self.last_scan, -math.pi/3)
            distance_right = self.get_scan_distance(self.last_scan, math.pi/3)
            distance_forward = self.get_scan_distance(self.last_scan, 0)
            distance_side = (self.last_scan.ranges[0], self.last_scan.ranges[-1])

            # stuck against wall
            if self.immobile:
                move_cmd.linear.x = 1
                self.move(move_cmd)
                # stuck against pole
                if distance_forward < 0.3:
                    move_cmd.linear = Vector3(-0.5, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, 1.0)
                    self.move_cycles = 5
                elif not self.tried_backward:
                    move_cmd.linear = Vector3(-0.5, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, 0.0)
                    self.move_cycles = 5
                    self.tried_backward = True
                else:
                    move_cmd.linear = Vector3(0.5, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, 0.0)
                    self.move_cycles = 5
                    self.tried_backward = False

            elif distance_left < 0.6 and distance_right < 0.6:
                # stuck in cubby
                if distance_forward < 2:
                    move_cmd.linear = Vector3(-1.0, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, 1.0)
                    self.move_cycles = 10
                    rospy.logwarn(f"{self.namespace} stuck in cubby")
                # inside narrow corridor
                elif distance_left < distance_right:
                    move_cmd.linear = Vector3(0.25, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, 1.0)
                else:
                    move_cmd.linear = Vector3(0.25, 0.0, 0.0)
                    move_cmd.angular = Vector3(0.0, 0.0, -1.0)
            elif distance_left < 0.6:
                move_cmd.linear = Vector3(0.0, 0.0, 0.0)
                move_cmd.angular = Vector3(0.0, 0.0, 1.0)
            elif distance_right < 0.6:
                move_cmd.linear = Vector3(0.0, 0.0, 0.0)
                move_cmd.angular = Vector3(0.0, 0.0, -1.0)
            else:
                move_cmd.linear = Vector3(0.5, 0.0, 0.0)
                move_cmd.angular = Vector3(0.0, 0.0, 0.0)

            self.move(move_cmd)

    def move(self, move_cmd: Twist):
        self.cmd_vel_publisher.publish(move_cmd)

    def _ground_truth_pose_callback(self, odom):
        new_pose = odom.pose.pose
        if new_pose == self.pose:
            self.immobile = True
        else:
            self.immobile = False
            self.pose = odom.pose.pose

    def _base_scan_callback(self, scan):
        self.last_scan = scan

    def get_scan_distance(self, scan, angle):
        assert (angle > scan.angle_min and angle < scan.angle_max)
        i = int((angle - scan.angle_min) / scan.angle_increment)
        return scan.ranges[i]


def get_world_coord(vector, res):
    x = (vector[0] * res)
    y = (vector[1] * res)
    return [x, y]


def get_grid_coord(pose, origin, res):
    x = (pose.x - origin[0]) / res
    y = (pose.y - origin[1]) / res
    return x, y


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


if __name__ == "__main__":
    try:
        rospy.init_node("move_controller")
        params = rospy.get_param("~namespace")
        rospy.sleep(2)
        Controller(params)
    except Exception as e:
        print(traceback.print_exc())