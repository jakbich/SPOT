#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import ros_numpy
import numpy as np
import tf
from message_filters import TimeSynchronizer, Subscriber

class PlaneSegmentation:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('plane_segmentation')

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.right_camera_source = rospy.get_param("/right_camera/source_frame")
        self.right_camera_target = rospy.get_param("/right_camera/target_frame")
        self.left_camera_source = rospy.get_param("/left_camera/source_frame")
        self.left_camera_target = rospy.get_param("/left_camera/target_frame")
        self.segmentation_treshold = rospy.get_param("/segmentation_treshold")
        update_rate = rospy.get_param("/update_rate")
        
        self.rate = rospy.Rate(update_rate)

        # Get fixed transformation matrix between cameras and base of robot
        self.right_trans = self.get_transformation(str(self.right_camera_source),str(self.right_camera_target))
        self.left_trans = self.get_transformation(str(self.left_camera_source),str(self.left_camera_target))

        # Set up subscriber and publisher
        sub_right = Subscriber("/spot/depth/frontright/depth_in_visual", PointCloud2, queue_size=2)
        sub_left = Subscriber("/spot/depth/frontleft/depth_in_visual", PointCloud2, queue_size=2)
        self.pub_non_ground_points = rospy.Publisher("/spot/depth/plane_segmentation/non_ground", PointCloud2)
        self.pub_ground_points = rospy.Publisher("/spot/depth/plane_segmentation/ground", PointCloud2)

        # Synchronize the subscribers based on their timestamps
        ts = TimeSynchronizer([sub_right, sub_left], 5)
        ts.registerCallback(self.callback)

    
    # Run the node on the specified rate
    def run(self) -> None:
        while not rospy.is_shutdown():
            self.rate.sleep()


    # Synchronized callback
    def callback(self, right_msg, left_msg) -> None:
        # Read synchronised time
        time = right_msg.header.stamp

        # Make from the input PointCloud2 message a numpy array
        pc_right = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(right_msg)
        pc_left = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(left_msg)

        # Make pointcloud homogeneous
        pc_right_hompgenous = np.hstack((pc_right, np.ones(len(pc_right)).reshape(-1, 1)))
        pc_left_hompgenous = np.hstack((pc_left, np.ones(len(pc_left)).reshape(-1, 1)))

        # Apply transformation and convert back to normal coordinates
        pc_right_hompgenous = np.dot(self.right_trans, pc_right_hompgenous.T).T
        pc_right = pc_right_hompgenous[:, :3] / pc_right_hompgenous[:, -1:]

        pc_left_hompgenous = np.dot(self.left_trans, pc_left_hompgenous.T).T
        pc_left = pc_left_hompgenous[:, :3] / pc_left_hompgenous[:, -1:]

        # Combine both pointclouds into one array
        points = np.vstack((pc_right, pc_left))

        # Filter points that are out of reach
        distances = np.linalg.norm(points, axis=1)
        angles = np.arctan(points[:, 0] / points[:, 1])
        in_reach = np.where((distances <= 4.5) & (distances >= 0.7) & (abs(angles) > np.pi / 4))[0]
        points = points[in_reach]

        # Ground plane segmentation based on Z-coordinate
        non_ground_points = points[np.where(points[:, 2] > self.segmentation_treshold)]
        ground_points = points[np.where(points[:, 2] <= self.segmentation_treshold)]

        # Create PointCloud2 message from array
        header = Header()
        header.stamp = time
        header.frame_id = str(self.left_camera_target)[1:]

        msg_non_ground_points = point_cloud2.create_cloud_xyz32(header, non_ground_points)
        msg_ground_points = point_cloud2.create_cloud_xyz32(header, ground_points)

        # Publish the resulting pointcloud
        self.pub_non_ground_points.publish(msg_non_ground_points)
        self.pub_ground_points.publish(msg_ground_points)      

    def get_transformation(self, source: str, target: str) -> np.ndarray:
        trans_matrix = np.zeros((4, 4))
        position, quaternions = None, None

        # Get 
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(target, source, now, rospy.Duration(2.0))
            position, quaternions = self.tf_listener.lookupTransform(target, source, rospy.Time.now())
        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))
            
        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            trans_matrix = tf.transformations.euler_matrix(euler[0], euler[1], euler[2], axes='sxyz')

            trans_matrix[:3, 3] = np.array(position).T
            trans_matrix[3, 3] = 1

            return trans_matrix
        else:
            rospy.logfatal("Did not return")


if __name__ == '__main__':
    try:
        node = PlaneSegmentation()
        node.run()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
