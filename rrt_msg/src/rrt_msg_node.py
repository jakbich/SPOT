#!/usr/bin/env python

import math
import random
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import numpy as np
import tf
import matplotlib.pyplot as plt
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from message_filters import TimeSynchronizer, Subscriber

class RRT_planner:

    def __init__(self):

        rospy.init_node('rrt_msg_planning')

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up


        self.model_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.path_publisher = rospy.Publisher('/spot/planning/path', Path, queue_size=1)

        self.start_models = []
        self.goal_models = []
        self.start_positions = []
        self.goal_positions = []
        self.obstacle_positions = []

    def add_start_model(self, model_name):
        self.start_models.append(model_name)
    
    def add_goal_model(self, model_name):
        self.goal_models.append(model_name)
    
    def model_states_callback(self, msg):
        self.obstacle_models = []  # Initialize an empty list for obstacle models
        self.obstacle_positions = []  # Initialize an empty list for obstacle positions

        for model_name, model_pose in zip(msg.name, msg.pose):
            if model_name not in self.start_models and model_name not in self.goal_models:
                self.obstacle_models.append(model_name)
                self.obstacle_positions.append((model_pose.position.x,model_pose.position.y))
            
        print("Obstacle positions:", self.obstacle_positions)

        if self.start_models and self.goal_models and not self.start_positions and not self.goal_positions:
            for start_model in self.start_models:
                try:
                    start_index = msg.name.index(start_model)
                    start_position = msg.pose[start_index].position
                    self.start_positions.append((start_position.x, start_position.y))
                except ValueError:
                    print("Start model", start_model, "not found in model list.")
            
            for goal_model in self.goal_models:
                try:
                    goal_index = msg.name.index(goal_model)
                    goal_position = msg.pose[goal_index].position
                    self.goal_positions.append((goal_position.x, goal_position.y))
                except ValueError:
                    print("Goal model", goal_model, "not found in model list.")

            start = (self.start_positions[0][0], self.start_positions[0][1])
            goal = (self.goal_positions[0][0], self.goal_positions[0][1])

            path = self.rrt_planning(start, goal)

            if path:
                print("Path found!")
                for point in path:
                    print(point)
            else:
                print("Failed to find a path.")

            # Assuming the path variable contains the list of point
            # Extract x and y coordinates from the path
            if path is not None:
                x = [point[0] for point in path]
                y = [point[1] for point in path]
                plt.plot(x, y, 'r-')
                plt.plot(x[0], y[0], 'go')  # Mark the start point as green circle
                plt.plot(x[-1], y[-1], 'bo')  # Mark the goal point as blue circle
                for i in range(len(self.obstacle_positions)):
                    model_position = self.obstacle_positions[i]
                    model_width = 1
                    model_height = 1
                    plt.plot(model_position[0], model_position[1], 'o', markersize=10 * max(model_width, model_height), mew=2)
            else:
                print("Failed to find a valid path.")

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
            print("Path published!")

            # Plot the path
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('RRT Path with known models')
            plt.xlim(-5, 5)
            plt.ylim(-5, 5)
            plt.grid(True)
            plt.gca().invert_xaxis()
            plt.gca().invert_yaxis()
            plt.show()

            print("\nDone")


    def rrt_planning(self, start, goal, max_iter=10000, step_size=0.01):
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
                start[1] + (end[1] - start[1]) * scale
            )
            return new_point
        else:
            return end

    def is_point_valid(self, point):
        # Check if the given point is valid (e.g., not colliding with obstacles)
        # Implement your own obstacle detection and avoidance logic here
        # Return True if the point is valid (not colliding), False otherwise
        # Example implementation:
        # Check if the point is within an obstacle bounding box

        for obstacle in self.obstacle_positions:
            obstacle_x = obstacle[0]
            obstacle_y = obstacle[1]
            obstacle_width = 1  # Replace with the actual obstacle width
            obstacle_height = 1  # Replace with the actual obstacle height

            if (point[0] >= obstacle_x - obstacle_width / 2 and
                    point[0] <= obstacle_x + obstacle_width / 2 and
                    point[1] >= obstacle_y - obstacle_height / 2 and
                    point[1] <= obstacle_y + obstacle_height / 2):
                return False

        return True

    def is_goal_reached(self, point, goal):
        # Check if the goal has been reached from the given point in 2D
        distance = math.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
        return distance < 1  # Define a threshold for reaching the goal

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
        rrt.add_start_model('/')
        rrt.add_goal_model('cup_green')
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node rrt_msg could not be launched")
        pass