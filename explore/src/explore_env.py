# !/usr/bin/env python

# source: https://github.com/ctsaitsao/turtlebot3-slam/blob/main/nodes/exploration

import rospy
import actionlib
import numpy as np
import sys
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
from std_srvs.srv import Trigger, TriggerResponse

from explore.msg import ExploreFrontiersAction, ExploreFrontiersGoal

np.set_printoptions(threshold=sys.maxsize)  # for debugging


class Exploration:
    def __init__(self):
        # Initialize server
        self.server = actionlib.SimpleActionServer('explore', ExploreFrontiersAction, self.explore, False)
        rospy.loginfo("Waiting for 'explore' action server...")
        self.server.start()
        rospy.loginfo("Exploration server is up.")
        self.pub_goal = rospy.Publisher("/spot/mapping/active_slam/goal", Marker, queue_size=3)


        rospy.loginfo("Waiting for RRT path server.")
        self.rrt_path_client = actionlib.SimpleActionClient('rrt_path', MoveBaseAction)
        self.rrt_path_client.wait_for_server()
        rospy.logwarn("RRT path server is up.")

        # Initialize client
    def image_gradient(self, I, operator='Sobel'):
        """ Outputs an image's gradient based on one of three operators.

            Args:
               I (np.ndarray): input image
               operator (str): type of kernel to convolve image with
        """
        mag = np.zeros((I.shape[0], I.shape[1]))
        theta = np.zeros((I.shape[0], I.shape[1]))

        allowable_operators = ['Sobel', 'Roberts_Cross', 'Prewitt']
        if operator not in allowable_operators:
            raise ValueError(f'operator must be in {allowable_operators}')

        if operator == 'Sobel':
            Gx = np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])
            Gy = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])
        elif operator == 'Roberts_Cross':
            Gx = np.array([[1, 0], [0, -1]])
            Gy = np.array([[0, -1], [1, 0]])
        else:
            Gx = np.array([[-1, 0, 1], [-1, 0, 1], [-1, 0, 1]])
            Gy = np.array([[1, 1, 1], [0, 0, 0], [-1, -1, -1]])

        for i in range(1, I.shape[0] - 2):
            for j in range(1, I.shape[1] - 2):
                if operator == 'Sobel' or operator == 'Prewitt':
                    sub_img = I[i - 1:i + 2, j - 1:j + 2]
                elif operator == 'Roberts_Cross':
                    sub_img = I[i - 1:i + 1, j - 1:j + 1]
                Ix = np.sum(Gx * sub_img)
                Iy = np.sum(Gy * sub_img)
                mag[i, j] = math.sqrt(Ix**2 + Iy**2)
                theta[i, j] = math.atan2(Iy, Ix)

        return mag, theta

    def explore(self, goal):
        """ Autonomous exploration. Sends pose goals to move_base. """
        rospy.logwarn("Starting exploration.")
        while True:

            # Map data:
            map_data = rospy.wait_for_message("/spot/mapping/occupancy_grid", OccupancyGrid)
            rospy.logwarn("Received map data.")
            robot_pos = rospy.wait_for_message("/odom/ground_truth", Odometry)
            rospy.logwarn("Received robot position.")
            robot_pos = np.array([robot_pos.pose.pose.position.x, robot_pos.pose.pose.position.y, robot_pos.pose.pose.position.z])

            map = np.array(map_data.data)
            map_width = map_data.info.width
            map_height = map_data.info.height
            map_resolution = map_data.info.resolution
            map_origin = np.array([map_data.info.origin.position.x, map_data.info.origin.position.y]) 

            map = np.array(map).reshape((map_height, map_width))

            map_proccesed = map.copy()
            map_proccesed[map > 80] = 300
            map_proccesed[map < 0] = 300

            # Image gradient:
            laplacian, _ = self.image_gradient(map_proccesed, operator='Roberts_Cross')
            laplacian *= 255 / np.amax(laplacian)

            laplacian_normlized = (100 * (laplacian / laplacian.max())).astype(int)
            gradient_indices = np.array(np.where((laplacian_normlized > 75) & (map < 30))).T

            index = np.random.choice(gradient_indices.shape[0])
            map_x, map_y = gradient_indices[index]

            rospy.logwarn("map x: " + str(map_x) + " map y: " + str(map_y))

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"  # has to be in map frame
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = map_origin[1] + map_y * map_resolution
            goal.target_pose.pose.position.y = map_origin[0] + map_x * map_resolution
            goal.target_pose.pose.orientation.w = 1.0

            print("Goal location:", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            print("Gradients:", laplacian_normlized.min(), laplacian_normlized.max())

            
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
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            self.pub_goal.publish(marker)
            rospy.logwarn(marker.pose.position.x)

            # Transform odom coordinates to map coordinates:
            goal.target_pose.pose.position.x = map_x
            goal.target_pose.pose.position.y = map_y
            # Set z to one cto omit last path point
            goal.target_pose.pose.position.z = 1


            self.rrt_path_client.send_goal(goal)
            self.rrt_path_client.wait_for_result()
            result = self.rrt_path_client.get_result()

            if result:
                rospy.loginfo("Goal reached!")
            else:
                rospy.loginfo("Failed to reach goal!")
                self.rrt_path_client.cancel_goal()


            # map_msg = OccupancyGrid()
            # map_msg.header.frame_id = "odom"
            # map_msg.info.resolution = map_resolution
            # map_msg.info.width = int(map_width / map_resolution)
            # map_msg.info.height = int(map_height / map_resolution)
            # map_msg.info.origin.position.x = map_origin[0]
            # map_msg.info.origin.position.y = map_origin[1]
            # map_msg.data = laplacian_normlized.flatten()
            # map_msg.header.stamp = rospy.Time.now()
            # self.pub_cost_map.publish(map_msg)

            # fig, ax = plt.subplots()
            # im = ax.imshow(laplacian_normlized, cmap='hot', interpolation='nearest')
            # ax.invert_xaxis()
            # cbar = ax.figure.colorbar(im, ax = ax)
            # cbar.ax.set_ylabel("Color bar", rotation = -90, va = "bottom")
            # plt.show()

            break
        return True


def main():
    """ The main() function. """
    rospy.init_node('explore_node')
    exploration = Exploration()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass