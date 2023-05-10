#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import ros_numpy
import numpy as np
import tf
import math
from nav_msgs.msg import OccupancyGrid
from message_filters import TimeSynchronizer, Subscriber

class OccupancyMap:
    def __init__(self, size: np.ndarray, resolution: float, origin: np.ndarray) -> None:
        self.size = (size / resolution).astype(int)
        self.resolution = resolution
        self.grid = np.ones(self.size[0] * self.size[1], dtype=int) * -1
        self.origin = np.array([origin.x, origin.y])

        self.map_frame = rospy.get_param("/map_frame")
    
    def update(self, non_ground_points: np.ndarray, ground_points: np.ndarray, robot_location: np.ndarray, yaw: float) -> None:
        for i, points in enumerate([ground_points, non_ground_points]):
            points = points[:, :2] # Discard z-coordinate

            # Transform points into global (odom) frame
            rot_matrix = np.array([[math.cos(yaw),  -math.sin(yaw), robot_location[0]], 
                                   [math.sin(yaw),  math.cos(yaw),  robot_location[1]],
                                   [0,              0,              1]])
            points_homogenous = np.hstack((points, np.ones(len(points)).reshape(-1, 1)))
            points_homogenous = np.dot(rot_matrix, points_homogenous.T).T
            points = points_homogenous[:, :2] / points_homogenous[:, -1:]

            # Transform points to indices in the occupancy map
            points = (points - self.origin) // self.resolution
            indices = (points[:, 1] * self.size[0] + points[:, 0]).astype(int)
            # indices, counts = np.unique(indices, return_counts=True)

            # Initialize indices that are picked for the first time
            mask = self.grid == -1            
            idx = np.where(mask)[0] # Find the indices where mask is true
            initialization = np.intersect1d(idx, indices) # Get the indices that require initialization
            self.grid[initialization] = 50

            # Fill occupancy map with the new point cloud data
            if i == 0: # Ground points
                self.grid[indices] -= 10
            else:
                self.grid[indices] += 10


            self.grid[self.grid < -1] = 0
            self.grid[self.grid > 100] = 100

        # # Subsample grid
        # self.grid = self.grid.reshape(self.size)
        # conv_gird = self.grid[corner1_ji[0]:corner2_ji[0]+1, corner1_ji[1]:corner2_ji[1]+1]
        # rospy.logwarn("Array shape: (%i, %i)", self.grid.shape[0], self.grid.shape[1])
        # # Define convolution (https://stackoverflow.com/questions/43086557/convolve2d-just-by-using-numpy)
        # conv_gird = np.pad(conv_gird, 1, mode='constant')
        # view_shape = tuple((3, 3, conv_gird.shape[0], conv_gird.shape[1]))
        # strides = conv_gird.strides + conv_gird.strides
        # sub_matrices = np.lib.stride_tricks.as_strided(conv_gird, view_shape, strides)
        
        # # Apply mean convolution filter
        # conv_filter = np.ones((3, 3)) * (1/9)
        # result = np.einsum('ij,ijkl->kl',conv_filter,sub_matrices)
        # rospy.logwarn("Array shape: (%i, %i)", result.shape[0], result.shape[1])
        # self.grid[corner1_ji[0]:corner2_ji[0]+1, corner1_ji[1]:corner2_ji[1]+1] = result
        # self.grid = self.grid.flatten()  



class GridMapping:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('grid_map')

        self.occupancy_map = None
        self.prev_robot_pos = np.array([np.inf, np.inf])
        self.prev_robot_yaw = np.inf

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame[1:]
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size[0] / self.map_resolution)
        self.map_msg.info.height = int(self.map_size[1] / self.map_resolution)
        self.map_msg.info.origin.position.x = -self.map_size[0] / 2
        self.map_msg.info.origin.position.y = -self.map_size[1] / 2

        # Set up subscriber and publisher
        sub_non_ground_points = Subscriber("/spot/depth/plane_segmentation/non_ground", PointCloud2, queue_size=5)
        sub_ground_points = Subscriber("/spot/depth/plane_segmentation/ground", PointCloud2, queue_size=5)
        self.pub = rospy.Publisher("/spot/mapping/occupancy_grid", OccupancyGrid)

        # Synchronize the subscribers based on their timestamps
        ts = TimeSynchronizer([sub_non_ground_points, sub_ground_points], 5)
        ts.registerCallback(self.update_map)


    def update_map(self, msg_non_ground_points, msg_ground_points) -> None:
        time = msg_non_ground_points.header.stamp

        # Get position of the robot in map frame (odom)
        position, quaternions = None, None
        try:
            position, quaternions = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
            position = np.array(position)
            quaternions = np.array(quaternions)

        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))

        if position is not None and quaternions is not None:
            # Get euler angles from quaternions
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            yaw = euler[-1]

            # Check if robot has moved
            # if np.linalg.norm(position[:2] - self.prev_robot_pos) > 0.1 or abs(yaw - self.prev_robot_yaw) > np.pi / 36:            
                # Check if map is initialized
            if self.occupancy_map is None:
                self.occupancy_map = OccupancyMap(self.map_size, self.map_resolution, self.map_msg.info.origin.position)

            non_ground_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg_non_ground_points)
            ground_points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg_ground_points)

            self.occupancy_map.update(non_ground_points, ground_points, position[:2], yaw)

            # Publish occupancy map
            self.map_msg.data = self.occupancy_map.grid
            self.map_msg.header.stamp = time
            self.pub.publish(self.map_msg)

            self.prev_robot_pos = position[:2]
            self.prev_robot_yaw = yaw

if __name__ == '__main__':
    try:
        GridMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
