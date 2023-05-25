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
from std_msgs.msg import Header  # Import the Header class


import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
from rrt_msg.msg import MotionAction, MotionGoal, MotionFeedback, MotionResult


class RRTPath:
    """"
    Class for RRT path planning algorithm
    Input: 
        start: start position
        goal: goal position
        occypy_grid: occupancy grid

    Output:
        path: list of MoveBaseGoal
    Usage:
        rrt_path = RRTPath(start, goal, occupancy_grid).path

        if rrt_path is not None:
            --> code
    """

    def __init__(self, start, goal, grid) -> None:
        # Initialize start and goal position
        self.start_pos = start[:2]
        self.start_yaw = start[2]

        self.goal_pos = goal[:2]
        self.goal_yaw = goal[2]

        self.grid = grid

        self.robot_margin = 3 # robot margin for collision avoidance in grid cells

        # Initialize results
        self.path = None



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
        x = random.uniform(0, self.grid.shape[0])
        y = random.uniform(0 , self.grid.shape[1])

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
        if not self.map:
            return False

        map_data = self.map_data
        map_width = self.map_width
        map_height = self.map_height
        map_resolution = self.map_resolution
        map_origin_x = self.map_origin_x
        map_origin_y = self.map_origin_y

        if (point[0] < map_origin_x or
                point[1] < map_origin_y or
                point[0] > map_origin_x + map_width * map_resolution or
                point[1] > map_origin_y + map_height * map_resolution):
            return False

        map_x = int((point[0] - map_origin_x) / map_resolution)
        map_y = int((point[1] - map_origin_y) / map_resolution)
        map_index = map_y * map_width + map_x

        if map_data[map_index] > 50 or map_data[map_index] < 0:
            return False

        return True

    def is_goal_reached(self, point, goal):
        # Check if the goal has been reached from the given point in 2D
        distance = math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
        angle_diff = abs(point[2] - math.atan2(goal[1] - point[1], goal[0] - point[0]))
        return distance < 1 and angle_diff < math.radians(10)  # Define thresholds for reaching the goal position and orientation

    def generate_path(self, tree, start, end):
        # Generate the path from start to end using the tree dictionary
        path = [end]
        current = end
        while current != start:
            current = tree[current]
            path.append(current)

        path.reverse()
        return path
        



class RRT_planner:

    def __init__(self) -> None:

        rospy.init_node('rrt_msg_planning')

        self.map = None  # Initialize the 'map' attribute to None
        self.start_models = []
        self.goal_models = []
        self.start_positions = []
        self.goal_positions = []
        self.robot_positions = None
        self.robot_pos = None
        self.robot_yaw = None

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        self.grid_location_msg = Float64MultiArray()

        # Define robot dimensions for collision avoidance
        self.robot_width = 1.0
        self.robot_height = 1.0

        self.origin_x = -self.map_size[0]/2
        self.origin_y = -self.map_size[1]/2
        self.width = int(self.map_size[0]/self.map_resolution)
        self.height = int(self.map_size[1]/self.map_resolution)

        #Setup subscriber
        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.robot_pos_sub = rospy.Subscriber("/odom/ground_truth", Odometry, self.odom_callback, queue_size=5)
        self.map_sub = rospy.Subscriber('/spot/mapping/map', OccupancyGrid, self.map_callback)
        # self.map_sub = rospy.Subscriber('/spot/mapping/occupancy_grid"', OccupancyGrid, self.map_callback)

        # occupancy_grid_sub = rospy.Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, self.grid_callback, queue_size=5)
        robot_pos_sub = rospy.Subscriber("/spot/mapping/grid_location", Float64MultiArray, self.robot_pos_callback, queue_size=5)

        #Setup publisher
        self.path_publisher = rospy.Publisher('/spot/planning/path', Path, queue_size=1)
        self.move_base_publisher = rospy.Publisher('/move_base/goal', MoveBaseAction, queue_size=1)

        
        self.__move_base_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        self.__move_base_client.wait_for_server()



    def odom_callback(self, msg):
        time = msg.header.stamp

        # Get robot position
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        quaternions = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    
    # callback function for occupancy grid 
    def grid_callback(self, msg):
        rospy.logwarn("Occupancy grid callback")
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # callback function for odometry
    def robot_pos_callback(self, msg): 
        rospy.logwarn("Robot position callback")
        self.robot_pos = msg.data[:2]
        self.yaw = msg.data[2]

    def map_callback(self, map_msg):
        # Access map information from the received message
        self.map = map_msg  # Set the 'map' attribute with the received message
        self.map_width = map_msg.info.width
        self.map_height = map_msg.info.height
        self.map_resolution = map_msg.info.resolution
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        self.map_data = map_msg.data

    def add_start_model(self, model_name):
        self.start_models.append(model_name)
    
    def add_goal_model(self, model_name):
        self.goal_models.append(model_name)

    def model_states_callback(self, msg):

        self.extract_start_positions(msg)
        self.extract_goal_positions(msg)

        # start_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        # start_yaw = np.array([msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # start = np.concatenate((start_position, start_yaw))


        # start_x, start_y = self.robot_positions
        # start = (start_x, start_y)

        # start_y = self.robot_positions[0][1]
        # start = (start_x, start_y)
        # print(self.robot_pos)
    
        # grid_position = self.robot_pos  # Replace x_grid and y_grid with the desired grid position
        # relative_position = np.array(grid_position) * self.map_resolution
        # start = relative_position + np.array([self.origin_x, self.origin_y])
        print(self.start_models.append('/'))

        print(self.goal_models.append('cup_green'))

        start = (self.start_positions[0][0], self.start_positions[0][1])
        goal = (self.goal_positions[0][0], self.goal_positions[0][1])

        print(start)
        print(goal)

        rospy.logwarn("path =" +str(start))

        path = self.rrt_planning(start, goal)

        if path is not None:
            print("Path found!")
            self.print_path(path)
        else:
            print("Failed to find a path.")
            rospy.signal_shutdown("Shutting down node")  # Shut off the node

        # # Assuming the path variable contains the list of point
        # # Extract x and y coordinates from the path
        # if path is not None:
        #     x = [point[0] for point in path]
        #     y = [point[1] for point in path]
        #     plt.plot(x, y, 'r-')
        #     plt.plot(x[0], y[0], 'go')  # Mark the start point as green circle
        #     plt.plot(x[-1], y[-1], 'bo')  # Mark the goal point as blue circle
        #     # for i in range(len(self.obstacle_positions)):
        #     #     model_position = self.obstacle_positions[i]
        #     #     model_width = 1
        #     #     model_height = 1
        #     #     plt.plot(model_position[0], model_position[1], 'o', markersize=10 * max(model_width, model_height), mew=2)
        # else:
        #     print("Failed to find a valid path.")

        
        self.publish_path(path)
        print("Path published!")
        
        print("\nDone")

    def extract_start_positions(self, msg):
        for start_model in self.start_models:
            try:
                start_index = msg.name.index(start_model)
                start_position_real = msg.pose[start_index].position
                # relative_position = np.array([start_position_real.x,start_position_real.y]) - np.array([self.origin_x, self.origin_y])
                # start_position = [int(relative_position[0]/self.map_resolution), int(relative_position[1]/self.map_resolution)]
                # self.start_positions.append((start_position[0], start_position[1]))
                self.start_positions.append((start_position_real.x, start_position_real.y))
            except ValueError:
                print("Start model", start_model, "not found in model list.")

    # def start_position_grid(self, msg):
    #     start_grid = self.start_position_grid

    def extract_goal_positions(self, msg):
        for goal_model in self.goal_models:
            try:
                goal_index = msg.name.index(goal_model)
                goal_position_real = msg.pose[goal_index].position
                # print(goal_position_real)
                # relative_position = np.array([goal_position_real.x,goal_position_real.y]) - np.array([self.origin_x, self.origin_y])
                # goal_position = [int(relative_position[0]/self.map_resolution), int(relative_position[1]/self.map_resolution)]
                # self.goal_positions.append((goal_position[0], goal_position[1]))
                self.goal_positions.append((goal_position_real.x, goal_position_real.y))

            except ValueError:
                print("Goal model", goal_model, "not found in model list.")

    def print_path(self, path):
        # for point in path:
        #     print(point)
        # for i, point in enumerate(path):
        #     print(point)
        #     time.sleep(5)

        for i, metric_point in enumerate(path):
            relative_position = np.array(metric_point[:2]) - np.array([self.origin_x, self.origin_y])
            grid_point = [int(relative_position[0] / self.map_resolution), int(relative_position[1] / self.map_resolution)]
            # orientation = metric_point[2]
            print(grid_point)#, orientation)
            # time.sleep(0.5)

            
    def publish_path(self, path):
        if path is None:
            print("Path is None")
            return
        
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'odom'

        for point in path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0

            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

        # Create a MoveBaseAction message
        action_msg = MoveBaseAction()
        action_msg.header = Header()


        # Loop through the path and add each point as a goal to the action message
        for point in path:
            goal = MoveBaseGoal()
            goal.target_pose.header = action_msg.header
            goal.target_pose.header.frame_id = "odom"  # has to be in map frame
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = point[0]
            goal.target_pose.pose.position.y = point[1]
            goal.target_pose.pose.orientation.w = 0.0  # Assuming no rotation?
            action_msg.goals.append(goal)
            print(action_msg.goals.append(goal))

        # Publish the action message to the move_base action server
        # self.move_base_publisher.publish(action_msg)
        self.__move_base_client.send_goal(action_msg)

        self.__move_base_client.wait_for_result(timeout=rospy.Duration(10.0))
        result = self.__move_base_client.get_result()

        if result:
            rospy.loginfo("Goal reached!")
        else:
            rospy.loginfo("Failed to reach goal!")
            self.__move_base_client.cancel_goal()



    def rrt_planning(self, start, goal, max_iter=100000, step_size=0.5):
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
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
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
        if not self.map:
            return False

        map_data = self.map_data
        map_width = self.map_width
        map_height = self.map_height
        map_resolution = self.map_resolution
        map_origin_x = self.map_origin_x
        map_origin_y = self.map_origin_y

        if (point[0] < map_origin_x or
                point[1] < map_origin_y or
                point[0] > map_origin_x + map_width * map_resolution or
                point[1] > map_origin_y + map_height * map_resolution):
            return False

        map_x = int((point[0] - map_origin_x) / map_resolution)
        map_y = int((point[1] - map_origin_y) / map_resolution)
        map_index = map_y * map_width + map_x

        if map_data[map_index] > 50 or map_data[map_index] < 0:
            return False

        return True

    def is_goal_reached(self, point, goal):
        # Check if the goal has been reached from the given point in 2D
        distance = math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
        angle_diff = abs(point[2] - math.atan2(goal[1] - point[1], goal[0] - point[0]))
        return distance < 1 and angle_diff < math.radians(10)  # Define thresholds for reaching the goal position and orientation

    def generate_path(self, tree, start, end):
        # Generate the path from start to end using the tree dictionary
        path = [end]
        current = end
        while current != start:
            current = tree[current]
            path.append(current)

        path.reverse()
        return path

if __name__ == '__main__':
    try:
        rrt = RRT_planner()
        # rrt.add_start_model('/')
        rrt.add_goal_model('cup_green')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node rrt_msg could not be launched")
        rospy.signal_shutdown("Shutting down node")  # Shut off the node
