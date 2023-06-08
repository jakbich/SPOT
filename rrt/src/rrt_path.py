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
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult, MoveBaseActionResult, MoveBaseActionGoal


class Node:
    """"
    Node for RRT path planning algorithm
    """
    def __init__(self, x, y, parent=None) -> None:
        self.x = int(x)
        self.y = int(y)
        self.parent = parent

class RRT:
    """
    Class for RRT path planning algorithm
    """
    def __init__(self, start, goal, grid, max_iter=1000, step_size=10) -> None:
        self.start = Node(start[1], start[0])
        self.goal = Node(goal[0], goal[1])
        self.grid = grid
        self.nodes = []

        self.max_iter = int(max_iter)
        self.step_size = step_size

        self.collision_free_area = 3 # grid cells

        self.goal_proximity = 15 # grid distance to goal to consider goal reached

    # calculate distance between two points
    def get_distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)


    # generate random node 
    def generate_random_node(self):
        x = int(random.uniform(-self.grid.shape[0], self.grid.shape[0]))
        y = int(random.uniform(-self.grid.shape[1], self.grid.shape[1]))

        return Node(x, y)
    
    # find nearest node to random node
    def find_nearest_node(self, node):
        min_dist = float('inf')
        closest_node = None

        for n in self.nodes:
            if n.x != node.x and n.y != node.y:
                dist = self.get_distance(n, node)
                if dist < min_dist:
                    min_dist = dist
                    closest_node = n
        
        return closest_node
    
    # check if line between two nodes is collision free
    def is_collision_free(self, node1, node2):
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y

        # check if both nodes are valid
        if self.grid[int(x1), int(y1)] >= 50 or self.grid[int(x2), int(y2)] >= 50:
            return False
        elif self.grid[int(x1), int(y2)] < 0 or self.grid[int(x2), int(y1)] < 0:
            return False
        
        # Check if node2 has enough space around it for SPOT to turn
        if node2 != self.goal:
            for i in range(-self.collision_free_area, self.collision_free_area):
                for j in range(-self.collision_free_area, self.collision_free_area):
                    if self.grid[int(x2 + i), int(y2 + j)] >= 25 or self.grid[int(x2 + i), int(y2 + j)] < 0:
                        return False
        
        # check if line between nodes is collision free
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx < self.step_size and dy < self.step_size:
            if dx > dy:
                steps = int(dx)
            else:
                steps = int(dy)
        else:
            steps = int(self.step_size)
            
        # rospy.logwarn("steps: {}".format(steps))

        x_inc = dx / steps
        y_inc = dy / steps

        # rospy.logwarn("x_inc: {}".format(x_inc))
        # rospy.logwarn("y_inc: {}".format(y_inc))

        for i in range(steps):
            x = int(x1 + i * x_inc)
            y = int(y1 + i * y_inc)

            if self.grid[int(x), int(y)] >= 50 or self.grid[int(x), int(y)] < 0:
                return False
            # if self.grid[int(x), int(y)] >= 50:
            #     return False
        
        return True

    # extend tree towards random node
    def extend(self):
        random_node = self.generate_random_node()
        # rospy.logwarn("random x, y: {}, {}".format(random_node.x, random_node.y))
        nearest_node = self.find_nearest_node(random_node)
        if nearest_node is None:
            # rospy.logwarn("No nearest node found")
            return None
        # rospy.logwarn("nearest x, y: {}, {}".format(nearest_node.x, nearest_node.y))

        distance = self.get_distance(nearest_node, random_node)
        if distance > self.step_size:
            theta = math.atan2(random_node.y - nearest_node.y, random_node.x - nearest_node.x)
            x = int(nearest_node.x + self.step_size * math.cos(theta))
            y = int(nearest_node.y + self.step_size * math.sin(theta))
            new_node = Node(x, y)
        else:
            new_node = random_node

        if self.is_collision_free(nearest_node, new_node):
            new_node.parent = nearest_node
            self.nodes.append(new_node)
            return new_node
        
        return None
    
    # plan path from start to goal
    def plan_path(self):
        rospy.logwarn("Planning path")
        self.nodes.append(self.start)
        for _ in range(self.max_iter):
            rospy.logwarn("Iteration: {}".format(_))
            new_node = self.extend()
            if new_node is not None:
                if self.is_collision_free(new_node, self.goal):
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)

                    return self.get_path()

                if self.get_distance(new_node, self.goal) <= self.goal_proximity:
                    self.goal.parent = new_node
                    self.nodes.append(self.goal)

                    return self.get_path()

    # get path from start to goal
    def get_path(self):
        path = []
        node = self.goal

        while node.parent is not None:
            path.append((node.x, node.y))
            node = node.parent
        
        path.append((self.start.x, self.start.y))
        path.reverse()

        return path  

            


class RRTPath:
    def __init__(self) -> None:
        rospy.init_node('rrt_path')
        
        # Initialize server
        rospy.logwarn("Starting rrt path server")
        self.server = actionlib.SimpleActionServer('rrt_path', MoveBaseAction, self.run_rrt, False)
        self.server.start()
        

        # Initialize client
        self.motion_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)

        # Initialize variables
        self.margin = 15  # Margin for the grid
        self.grid = None
        self.rrt_path = []
        self.pos = None
        self.yaw = None
        self.goal_pos = None
        self.goal_yaw = None

        self.DEBUG = True

        # Create MoveBaseAction template
        self.move_base_goal_msg = MoveBaseGoal()
        self.move_base_goal_msg.target_pose.header.frame_id = "odom"
        

        # Setup publisher
        self.pub_debug_path = rospy.Publisher("/spot/planning/path_steps", Path, queue_size=3)
        self.pub_debug_marker = rospy.Publisher("/spot/planning/path_marker", Marker, queue_size=3)
        self.pub_debug_marker_2 = rospy.Publisher("/spot/planning/path_marker_2", Marker, queue_size=3)


    def run_rrt(self, goal: MoveBaseGoal):
        rospy.logwarn("Run RRT")
        self.goal_pos = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
        self.goal_yaw = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w], 'sxyz')[-1]  

        
        pos_msg = rospy.wait_for_message("/spot/mapping/grid_location", Float64MultiArray)
        rospy.logwarn("Received position")
        self.pos = pos_msg.data[:2]
        self.yaw = pos_msg.data[2]

        start_pos = [self.pos[0], self.pos[1]]

        # If the goal is too close to the robot, we just return
        if np.linalg.norm(np.array(start_pos) - np.array(self.goal_pos)) < 10:
            rospy.logwarn("Goal too close to robot")
            # self.server.set_succeeded()
            return    


        grid_msg = rospy.wait_for_message("/spot/mapping/occupancy_grid", OccupancyGrid)
        rospy.logwarn("Received grid")

        grid_width = grid_msg.info.width
        grid_height = grid_msg.info.height
        grid_resolution = grid_msg.info.resolution
        grid_origin = np.array([grid_msg.info.origin.position.x, grid_msg.info.origin.position.y]) 

        self.grid = np.array(grid_msg.data).reshape((grid_msg.info.height, grid_msg.info.width))

        if self.pos is None or self.grid is None:
            rospy.logwarn("No position or grid received")
            return
        
        # When the system is initialized, the grid beneath the robot
        # is valued -1, therefore the RRT will not initialize well
        # Therefore, when this is the case, we hard code so that the 
        # RRT is initialized a few grids in front of the robot using the yaw
        if self.grid[int(self.pos[1]), int(self.pos[0])] == -1:
            start_pos[0] += 10 * math.cos(self.yaw)
            start_pos[1] += 10 * math.sin(self.yaw)

        # Run RRT
        rospy.logwarn("Start RRT Algorithm")
        rrt = RRT(start_pos, self.goal_pos, self.grid)

        self.rrt_path = rrt.plan_path()
        if self.rrt_path is None:
            rospy.logwarn("No path found")
            rospy.sleep(rospy.Duration(5))
            return
        
        rospy.logwarn("Path: {}".format(self.rrt_path))
        
        # Debug 
        if self.DEBUG:
            rospy.logwarn("DEBUGGING")
            rospy.logwarn("Length of path: {}".format(len(self.rrt_path)))
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = rospy.Time.now()

            # Add the first point if the area underneath the robot was unkown:
            if self.grid[int(self.pos[1]), int(self.pos[0])] == -1:
                pose = PoseStamped()
                y, x = self.pos
                pose.pose.position.x = grid_origin[1] + y * grid_resolution
                pose.pose.position.y = grid_origin[0] + x * grid_resolution
                pose.pose.position.z = 0.0

                path_msg.poses.append(pose)

            for point in self.rrt_path:
                pose = PoseStamped()
                x, y = point
                pose.pose.position.x = grid_origin[1] + y * grid_resolution
                pose.pose.position.y = grid_origin[0] + x * grid_resolution
                pose.pose.position.z = 0.0

                path_msg.poses.append(pose)

            self.pub_debug_path.publish(path_msg)

            marker = Marker()
            # x_marker, y_marker = self.pos
            y_marker, x_marker = self.pos
            marker.header.frame_id = "odom"  # Modify the frame_id according to your needs
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = grid_origin[0] + y_marker * grid_resolution 
            marker.pose.position.y = grid_origin[1] + x_marker * grid_resolution
            marker.pose.position.z = 0.2
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

            self.pub_debug_marker.publish(marker)

            marker = Marker()
            # x_marker, y_marker = self.pos
            x_marker, y_marker = self.rrt_path[-2]
            marker.header.frame_id = "odom"  # Modify the frame_id according to your needs
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = grid_origin[0] + y_marker * grid_resolution 
            marker.pose.position.y = grid_origin[1] + x_marker * grid_resolution
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
            marker.color.g = 1.0
            marker.color.b = 0.0

            self.pub_debug_marker_2.publish(marker)
            

        # rospy.sleep(rospy.Duration(10))

        # Send path point for point to motion control action server
        self.motion_client.wait_for_server()
        rospy.logwarn("Motion server found")

        for i, point in enumerate(self.rrt_path):
            # Skip the last point
            if i == len(self.rrt_path) - 1:
                break

            action_goal = MoveBaseGoal()
            # action_goal.header.stamp = rospy.Time.now()
            # action_goal.header.frame_id = "odom"
            y, x = point # Mirrored

            # Location in the grid map
            action_goal.target_pose.pose.position.x = x
            action_goal.target_pose.pose.position.y = y

            # Send goal
            self.motion_client.send_goal(action_goal)
            self.motion_client.wait_for_result()
            result = self.motion_client.get_result()

            if result:
                rospy.logwarn("Point {} executed successfully".format(i))
            else:
                rospy.logwarn("Point {} execution failed".format(i))

            rospy.sleep(rospy.Duration(0.5))
        
        rospy.logwarn("Path executed successfully")

        # Return succes
        result = MoveBaseActionResult()
        result.header.stamp = rospy.Time.now()
        result.status.status = 0 # Set the status code, where 0 represents success
        result.status.text = "Path executed successfully"

        self.server.set_succeeded(result)


if __name__ == '__main__':
    try:
        rrt = RRTPath()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node rrt_path could not be launched")
        rospy.signal_shutdown("Shutting down node")  # Shut off the node
