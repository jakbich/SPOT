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

class OccupancyMap:
    def __init__(self, size: np.ndarray, resolution: float, origin: np.ndarray) -> None:
        self.size = (size / resolution).astype(int)
        self.resolution = resolution
        self.grid = np.ones(self.size[0] * self.size[1], dtype=int) * -1
        self.origin = np.array([origin.x, origin.y])

        self.map_frame = rospy.get_param("/map_frame")
    
    def update(self, robot_location: np.ndarray, yaw: float, pc: np.ndarray):
        # Transform points into global (odom) frame
        pc = pc[:, :2] # Discard z-coordinate
        rot_matrix = np.array([[math.cos(yaw), -math.sin(yaw), robot_location[0]], 
                               [math.sin(yaw), math.cos(yaw), robot_location[1]],
                               [0, 0, 1]])
        pc = np.hstack((pc, np.ones(len(pc)).reshape(-1, 1)))
        pc = np.dot(rot_matrix, pc.T).T
        pc = pc[:, :2] / pc[:, -1:]

        # Transform points to indices in the occupancy map
        pc = (pc - self.origin) // self.resolution
        indeces = (pc[:, 1] * 200 + pc[:, 0]).astype(int)
        self.grid[indeces] = int(100)


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

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_publish_freq = rospy.get_param("/map_publish_freq")
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
        self.sub = rospy.Subscriber("/spot/depth/pc_plane_segmentation", PointCloud2, self.update_map, queue_size=5)
        self.pub = rospy.Publisher("/spot/mapping/occupancy_grid", OccupancyGrid)


    def update_map(self, pc_msg) -> None:
        time = pc_msg.header.stamp

        position, quaternions = None, None
        try:
            position, quaternions = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
            position = np.array(position)
            quaternions = np.array(quaternions)

        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))

        if position is not None and quaternions is not None:
            # if np.linalg.norm(position[:2] - self.prev_robot_pos) > 0.1:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            yaw = euler[-1]

            if self.occupancy_map is None:
                self.occupancy_map = OccupancyMap(self.map_size, self.map_resolution, self.map_msg.info.origin.position)

            pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
            self.occupancy_map.update(position[:2], yaw, pc)

            # Publish occupancy map
            self.map_msg.data = self.occupancy_map.grid
            self.map_msg.header.stamp = time
            self.pub.publish(self.map_msg)

            self.prev_robot_pos = position[:2]

if __name__ == '__main__':
    try:
        GridMapping()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
