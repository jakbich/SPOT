#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import ros_numpy
import numpy as np
import tf
import math
from message_filters import TimeSynchronizer, Subscriber
from nav_msgs.msg import OccupancyGrid

class OccupancyMap:
    def __init__(self, size: np.ndarray, resolution: float, robot_position: np.ndarray) -> None:
        self.size = (size / resolution).astype(int)
        self.resolution = resolution
        self.grid = np.ones(self.size[0] * self.size[1]) * -1
        self.origin = robot_position - size // 2

        self.map_frame = rospy.get_param("/map_frame")

    # Converts map index to xy location in the world
    def to_xy(self, i: int, j: int) -> np.ndarray:
        x = i * self.resolution + self.center[0]
        y = j * self.resolution + self.center[1]
        return np.array([x, y])
    
    # Converts xy location in the world to index in map
    def to_ji(self, point: np.ndarray) -> np.ndarray:
        # Transform coordinate
        point = point - self.origin

        i = int(point[0] / self.resolution)
        j = int(point[1] / self.resolution)

        return np.array([j, i])
    
    # Converts xy location in the world to index in map
    def to_index(self, point: np.ndarray) -> int:
        j, i = self.to_ji(point)
        index = j * self.size[1] + i
        return int(index)
    
    def update(self, robot_location: np.ndarray, yaw: float, pc: np.ndarray):
        # Transform points into map frame
        pc = pc[:, :2] # Discard z-coordinate
        pc = pc + robot_location
        rot_matrix = np.array([[math.cos(yaw), 0], [0, math.sin(yaw)]])
        pc = np.dot(pc, rot_matrix)

        # Transform x-y points to indeces for flattened grid
        pc = pc - self.origin
        pc = pc / self.resolution
        indeces = (pc[:, 1] * self.size[1] + pc[:, 0]).astype(int)
        indeces, counts = np.unique(indeces, return_counts=True)

        corner1 = np.array([min(pc[:, 0]), min(pc[:, 1])])
        rospy.logwarn("Corner1: (%f, %f)", corner1[0], corner1[1])
        corner1_ji = self.to_ji(corner1)
        corner2 = np.array([max(pc[:, 0]), max(pc[:, 1])])
        rospy.logwarn("Corner1: (%f, %f)", corner2[0], corner2[1])
        corner2_ji = self.to_ji(corner2)
        
        self.grid = self.grid.reshape(self.size)
        self.grid[corner1_ji[0]:corner2_ji[0]+1, corner1_ji[1]:corner2_ji[1]+1] = 10
        self.grid = self.grid.flatten()
        self.grid[indeces] = 90

        # Subsample grid
        self.grid = self.grid.reshape(self.size)
        conv_gird = self.grid[corner1_ji[0]:corner2_ji[0]+1, corner1_ji[1]:corner2_ji[1]+1]
        rospy.logwarn("Array shape: (%i, %i)", self.grid.shape[0], self.grid.shape[1])
        # Define convolution (https://stackoverflow.com/questions/43086557/convolve2d-just-by-using-numpy)
        conv_gird = np.pad(conv_gird, 1, mode='constant')
        view_shape = tuple((3, 3, conv_gird.shape[0], conv_gird.shape[1]))
        strides = conv_gird.strides + conv_gird.strides
        sub_matrices = np.lib.stride_tricks.as_strided(conv_gird, view_shape, strides)
        
        # Apply mean convolution filter
        conv_filter = np.ones((3, 3)) * (1/9)
        result = np.einsum('ij,ijkl->kl',conv_filter,sub_matrices)
        rospy.logwarn("Array shape: (%i, %i)", result.shape[0], result.shape[1])
        self.grid[corner1_ji[0]:corner2_ji[0]+1, corner1_ji[1]:corner2_ji[1]+1] = result
        self.grid = self.grid.flatten()  


class GridMapping:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('grid_map')

        self.occupancy_map = None
        self.prev_robot_pos = np.array([np.inf, np.inf])

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_origin = np.array([-1, 2])
        self.map_publish_freq = rospy.get_param("/map_publish_freq")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size[0] / self.map_resolution)
        self.map_msg.info.height = int(self.map_size[1] / self.map_resolution)
        self.map_msg.info.origin.position.x = self.map_origin[0]
        self.map_msg.info.origin.position.y = self.map_origin[1]

        # Set up subscriber and publisher
        self.sub = rospy.Subscriber("/spot/depth/pc_plane_segmentation", PointCloud2, self.update_map, queue_size=5)
        self.pub = rospy.Publisher("/spot/mapping/occupancy_grid", OccupancyGrid)


    def update_map(self, pc_msg) -> None:
        time = pc_msg.header.stamp

        position, quaternions = None, None
        try:
            position, quaternions = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)

        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))

        if position is not None and quaternions is not None:
            if np.linalg.norm(position[:2] - self.prev_robot_pos) > 0.1:
                euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
                yaw = euler[-1]

                if self.occupancy_map is None:
                    self.occupancy_map = OccupancyMap(self.map_size, self.map_resolution, position[:2])

                pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
                self.occupancy_map.update(position[:2], yaw, pc)

                # Publish occupancy map
                self.map_msg.data = self.occupancy_map.grid
                self.map_msg.header.stamp = time
                self.pub(self.map_msg)

                self.prev_robot_pos = position[:2]

if __name__ == '__main__':
    try:
        GridMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
