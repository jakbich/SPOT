#!/usr/bin/env python

import math
import random
import rospy
from rospy.client import time
from std_msgs.msg import Header, Float64MultiArray
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Header 


import actionlib
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult, MoveBaseActionResult

class RRTPath:
    """"
    Node for RRT path planning algorithm
    Input: 
        start: start position
        goal: goal position
        occypy_grid: occupancy grid

    Publishes:
        path: list of MoveBaseGoal to trajectory planner node
    Usage:
        rrt_path = RRTPath(start, goal, occupancy_grid).path

        if rrt_path is not None:
            --> code
    """

    def __init__(self, start, goal, grid) -> None:
        rospy.init_node('rrt_path')
        
        # Initialize server
        self.server = actionlib.SimpleActionServer('rrt_path', MoveBaseAction, self.run_rrt, False)
        self.server.start()

        self.trajectory_client = actionlib.SimpleActionClient('trajectory', GoalStatusArray)
        self.trajectory_client.wait_for_server()

        # Initialize variables
        self.margin = 3  # Margin for the grid
        self.grid = None
        self.rrt_path = []
        self.pos = None
        self.yaw = None
        self.goal_pos = None
        self.goal_yaw = None

        self.DEBUG = True

        # Create MoveBaseAction template
        self.move_base_goal_msg = MoveBaseGoal()
        self.move_base_goal_msg.target_pose.header.frame_id = "map"
        

        # Setup publisher
        self.path_pub = rospy.Publisher("/spot/planning/path", GoalStatusArray, queue_size=1)
        self.pub_goal = rospy.Publisher("/spot/planning/path_steps", Marker, queue_size=3)


    def run_rrt(self, goal: MoveBaseGoal):
        rospy.logwarn("Run RRT")
        self.goal_pos = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
        self.goal_yaw = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w], 'sxyz')[-1]  

        
        pos_msg = rospy.wait_for_message("/spot/mapping/grid_location", Float64MultiArray)
        self.pos = pos_msg.data[:2]
        self.yaw = pos_msg.data[2]

        grid_msg = rospy.wait_for_message("/spot/mapping/grid", OccupancyGrid)
        self.grid = np.array(grid_msg.data).reshape((grid_msg.info.height, grid_msg.info.width))

        if self.pos is None or self.grid is None:
            rospy.logwarn("No position or grid received")
            return
        
        # Run RRT
        self.rrt_path = self.rrt_planning(self.pos, self.goal_pos)
        if self.rrt_path is None:
            rospy.logwarn("No path found")
            return
        
        # Debug 
        if self.DEBUG:
            for goal in self.rrt_path:
                marker = Marker()
                marker.header.frame_id = "odom"  # Modify the frame_id according to your needs
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = goal.target_pose.pose.position.x # Modify the coordinates as per your desired point
                marker.pose.position.y = goal.target_pose.pose.position.y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

                self.pub_debug.publish(marker)
        
        # Send path to client
        goal_status_array = GoalStatusArray()
        goal_status_array.header.stamp = rospy.Time.now()
        goal_status_array.status_list =  self.rrt_path
        
        self.trajectory_client.send_goal(goal_status_array)
        self.trajectory_client.wait_for_result()
        result = self.trajectory_client.get_result()

        if result:
            rospy.logwarn("Path executed successfully")

            # Return succes 
            result = MoveBaseResult()
            result.header.stamp = rospy.Time.now()
            result.status.status = 0 # Set the status code, where 0 represents success
            result.status.text = "Path executed successfully"

            self.server.set_succeeded(result)
        else:
            rospy.logwarn("Path execution failed")


        

    def rrt_planning(self, start, goal, max_iter=100000, step_size=1):
        # tree = {start: None}
        tree = {(start[0], start[1]): None}

        for _ in range(max_iter):
            rand_point = self.generate_random_point()
            nearest_point = self.find_nearest_point(tree, rand_point)
            new_point = self.extend(nearest_point, rand_point, step_size)

            if new_point and self.is_point_valid(new_point):
                tree[new_point] = nearest_point
                if self.is_goal_reached(new_point, goal):
                    return self.generate_path(tree, start, new_point)

        return None

    def generate_random_point(self):
        # Generate a random point within the workspace bounds in 2D
        x = int(random.uniform(0, self.grid.shape[0]))
        y = int(random.uniform(0 , self.grid.shape[1]))

        return (x, y)


    def find_nearest_point(self, tree, point):
        # Find the nearest point in the tree to the given point
        nearest_point = None
        min_distance = float('inf')

        for p in tree.keys():
            distance = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
            if distance < min_distance:
                min_distance = distance
                nearest_point = p

        return nearest_point        

    def extend(self, start, end, step_size):
        # Extend the start point towards the end point within the step size in 2D
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        if distance > step_size:
            scale = step_size / distance
            new_point = (
                start[0] + (end[0] - start[0]) * scale,
                start[1] + (end[1] - start[1]) * scale,
                math.atan2(end[1] - start[1], end[0] - start[0])  # Calculate yaw angle
            )
            return new_point
        else:
            return end + (math.atan2(end[1] - start[1], end[0] - start[0]),)  # Append yaw angle

    def is_point_valid(self, point):
        if not self.grid:
            return False
        
        grid_width = self.grid.shape[0]
        grid_height = self.grid.shape[1]

        # Check if the point is outside the grid bounds
        if point[0] < 0 or point[0] >= grid_width or point[1] < 0 or point[1] >= grid_height:
            return False
        
        if self.grid[int(point[0]), int(point[1])] > 50 or self.grid[int(point[0]), int(point[1])] < 0:
            return False
        
        return True

    def is_goal_reached(self, point, goal):
        # Check if the goal has been reached from the given point in 2D
        distance = math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
        angle_diff = abs(point[2] - math.atan2(goal[1] - point[1], goal[0] - point[0]))

        # Define thresholds for reaching the goal position and orientation
        return distance < self.margin and angle_diff < math.radians(10) 

    def generate_path(self, tree, start, end):
        # Generate the path from start to end using the tree dictionary
        path = []

        # Convert x,y coordinates to MoveBaseAction
        self.move_base_goal_msg.target_pose.pose.position.x = end[0]
        self.move_base_goal_msg.target_pose.pose.position.y = end[1]

        self.move_base_goal_msg.target_pose.header.stamp = rospy.Time.now()

        # UITWERKEN 
        # Convert the yaw angle to quaternion
        # quaternion = tf.transformations.quaternion_from_euler(0, 0, end[2], axes='sxyz')
        # self.move_base_goal_msg.target_pose.pose.orientation.x = quaternion[0]
        # self.move_base_goal_msg.target_pose.pose.orientation.y = quaternion[1]
        # self.move_base_goal_msg.target_pose.pose.orientation.z = quaternion[2]
        # self.move_base_goal_msg.target_pose.pose.orientation.w = quaternion[3]

        path.append(self.move_base_goal_msg)

        current = end
        while current != start:
            current = tree[current]
            self.move_base_goal_msg.target_pose.pose.position.x = current[0]
            self.move_base_goal_msg.target_pose.pose.position.y = current[1]

            self.move_base_goal_msg.target_pose.header.stamp = rospy.Time.now()

            # UITWERKEN 
            # Convert the yaw angle to quaternion
            # quaternion = tf.transformations.quaternion_from_euler(0, 0, current[2], axes='sxyz')
            # self.move_base_goal_msg.target_pose.pose.orientation.x = quaternion[0]
            # self.move_base_goal_msg.target_pose.pose.orientation.y = quaternion[1]
            # self.move_base_goal_msg.target_pose.pose.orientation.z = quaternion[2]
            # self.move_base_goal_msg.target_pose.pose.orientation.w = quaternion[3]

            path.append(self.move_base_goal_msg)
        
        path.reverse()
        return path
        


if __name__ == '__main__':
    try:
        rrt = RRTPath()
        rospy.logwarn("Starting rrt path server")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node rrt_path could not be launched")
        rospy.signal_shutdown("Shutting down node")  # Shut off the node
