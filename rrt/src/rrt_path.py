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

        self.goal_proximity = 15 # grid distance to goal to consider goal reached


    def get_distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    
    def generate_random_node(self):
        x = int(random.uniform(-self.grid.shape[0], self.grid.shape[0]))
        y = int(random.uniform(-self.grid.shape[1], self.grid.shape[1]))

        return Node(x, y)
    

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
    

    def is_collision_free(self, node1, node2):
        x1, y1 = node1.x, node1.y
        x2, y2 = node2.x, node2.y

        if self.grid[int(x1), int(y1)] >= 50 or self.grid[int(x2), int(y2)] >= 50:
            return False
        elif self.grid[int(x1), int(y2)] < 0 or self.grid[int(x2), int(y1)] < 0:
            return False
        
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
            y_marker, x_marker = start_pos
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
            action_goal = MoveBaseGoal()
            # action_goal.header.stamp = rospy.Time.now()
            # action_goal.header.frame_id = "odom"
            x, y = point

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
        
        rospy.logwarn("Path executed successfully")

        # Return succes
        result = MoveBaseActionResult()
        result.header.stamp = rospy.Time.now()
        result.status.status = 0 # Set the status code, where 0 represents success
        result.status.text = "Path executed successfully"

        self.server.set_succeeded(result)



        
        # # Send path to client
        # goal_status_array = GoalStatusArray()
        # goal_status_array.header.stamp = rospy.Time.now()
        # goal_status_array.status_list =  self.rrt_path
        
        # self.trajectory_client.send_goal(goal_status_array)
        # self.trajectory_client.wait_for_result()
        # result = self.trajectory_client.get_result()

        # if result:
        #     rospy.logwarn("Path executed successfully")

        #     # Return succes 
        #     result = MoveBaseResult()
        #     result.header.stamp = rospy.Time.now()
        #     result.status.status = 0 # Set the status code, where 0 represents success
        #     result.status.text = "Path executed successfully"

        #     self.server.set_succeeded(result)
        # else:
        #     rospy.logwarn("Path execution failed")




        

    # def rrt_planning(self, start, goal, max_iter=10, step_size=5):
    #     # tree = {start: None}
    #     tree = {(start[0], start[1]): None}

    #     for _ in range(max_iter):
    #         rand_point = self.generate_random_point(tree)
    #         # rospy.logwarn("Random point: {}".format(rand_point))
    #         nearest_point = self.find_nearest_point(tree, rand_point)
    #         rospy.logwarn("Nearest point: {}".format(nearest_point))
    #         new_point = self.extend(nearest_point, rand_point, step_size)
    #         rospy.logwarn("New point: {}".format(new_point))

    #         # rospy.logwarn(self.is_point_valid(new_point))

    #         if new_point and self.is_point_valid(new_point):
    #             tree[new_point] = nearest_point
    #             # rospy.logwarn("New point added to tree: {}".format(new_point))
    #             if self.is_goal_reached(new_point, goal):
    #                 rospy.logwarn("Goal reached")
    #                 return self.generate_path(tree, start, new_point)

    #     return None

    # def generate_random_point(self, tree):
    #     # Generate a random point around the last point of the tree within a certain radius
    #     last_point = list(tree.keys())[-1]
    #     last_x = last_point[0]
    #     last_y = last_point[1]
    #     rospy.logwarn("Last point: {}".format(last_point))
    #     x = int(random.uniform(last_x - 10, last_x + 10))
    #     y = int(random.uniform(last_y - 10, last_y + 10))
    #     rospy.logwarn("Random point: {}".format((x, y)))

    #     return (x, y)


    # def find_nearest_point(self, tree, point):
    #     # Find the nearest point in the tree to the given point
    #     nearest_point = None
    #     min_distance = float('inf')

    #     for p in tree.keys():
    #         # rospy.logwarn("Point: {}".format(p))
    #         distance = math.sqrt((p[0] - point[0])**2 + (p[1] - point[1])**2)
    #         if distance < min_distance:
    #             min_distance = distance
    #             nearest_point = p

    #     return nearest_point        

    # def extend(self, start, end, step_size=1):
    #     # Extend the start point towards the end point within the step size in 2D
    #     new_point = (
    #         start[0] + end[0],
    #         start[1] + end[1],
    #         math.atan2(end[1] - start[1], end[0] - start[0])
    #     )

    #     return new_point
    
    #     # distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
    #     # if distance > step_size:
    #     #     scale = step_size / distance
    #     #     new_point = (
    #     #         start[0] + (end[0] - start[0]) * scale,
    #     #         start[1] + (end[1] - start[1]) * scale,
    #     #         math.atan2(end[1] - start[1], end[0] - start[0])  # Calculate yaw angle
    #     #     )
    #     #     return new_point
    #     # else:
    #     #     return end + (math.atan2(end[1] - start[1], end[0] - start[0]),)  # Append yaw angle

    # def is_point_valid(self, point):
    #     # rospy.logwarn("Checking if point is valid")
    #     if self.grid is None:
    #         # rospy.logwarn("Grid is None")
    #         return False
        
    #     grid_width = self.grid.shape[0]
    #     grid_height = self.grid.shape[1]

    #     # Check if the point is outside the grid bounds
    #     if point[0] < 0 or point[0] >= grid_width or point[1] < 0 or point[1] >= grid_height:
    #         # rospy.logwarn("Point is outside grid bounds")
    #         return False
        
    #     if self.grid[int(point[0]), int(point[1])] > 50:
    #         # rospy.logwarn("Point is in obstacle")
    #         return False
    #     # if self.grid[int(point[0]), int(point[1])] > 50 or self.grid[int(point[0]), int(point[1])] < 0:
    #     #     rospy.logwarn("Point is in obstacle")
    #     #     return False
        
    #     return True

    # def is_goal_reached(self, point, goal):
    #     # Check if the goal has been reached from the given point in 2D
    #     distance = math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
    #     # print(point, goal)
    #     # rospy.logwarn("Distance to goal: {}".format(distance))
    #     rospy.logwarn(distance < self.margin)
    #     angle_diff = abs(point[2] - math.atan2(goal[1] - point[1], goal[0] - point[0]))

    #     # Define thresholds for reaching the goal position and orientation
    #     return distance < self.margin

    # def generate_path(self, tree, start, end):
    #     # Generate the path from start to end using the tree dictionary
    #     path = []

    #     # Convert x,y coordinates to MoveBaseAction
    #     self.move_base_goal_msg.target_pose.pose.position.x = int(end[0])
    #     self.move_base_goal_msg.target_pose.pose.position.y = int(end[1])

    #     self.move_base_goal_msg.target_pose.header.stamp = rospy.Time.now()

    #     # UITWERKEN 
    #     # Convert the yaw angle to quaternion
    #     # quaternion = tf.transformations.quaternion_from_euler(0, 0, end[2], axes='sxyz')
    #     # self.move_base_goal_msg.target_pose.pose.orientation.x = quaternion[0]
    #     # self.move_base_goal_msg.target_pose.pose.orientation.y = quaternion[1]
    #     # self.move_base_goal_msg.target_pose.pose.orientation.z = quaternion[2]
    #     # self.move_base_goal_msg.target_pose.pose.orientation.w = quaternion[3]

    #     path.append(self.move_base_goal_msg)

    #     current = end
    #     while current != start:
    #         current = tree[current]
    #         self.move_base_goal_msg.target_pose.pose.position.x = int(current[0])
    #         self.move_base_goal_msg.target_pose.pose.position.y = int(current[1])

    #         self.move_base_goal_msg.target_pose.header.stamp = rospy.Time.now()

    #         # UITWERKEN 
    #         # Convert the yaw angle to quaternion
    #         # quaternion = tf.transformations.quaternion_from_euler(0, 0, current[2], axes='sxyz')
    #         # self.move_base_goal_msg.target_pose.pose.orientation.x = quaternion[0]
    #         # self.move_base_goal_msg.target_pose.pose.orientation.y = quaternion[1]
    #         # self.move_base_goal_msg.target_pose.pose.orientation.z = quaternion[2]
    #         # self.move_base_goal_msg.target_pose.pose.orientation.w = quaternion[3]

    #         path.append(self.move_base_goal_msg)
        
    #     path.reverse()
    #     return path
        


if __name__ == '__main__':
    try:
        rrt = RRTPath()
        rospy.logwarn("Starting rrt path server")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node rrt_path could not be launched")
        rospy.signal_shutdown("Shutting down node")  # Shut off the node
