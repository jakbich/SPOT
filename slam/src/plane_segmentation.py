#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import ros_numpy
import numpy as np
import tf

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
        self.update_rate = rospy.get_param("/update_rate")

        # Get fixed transformation matrix between cameras and base of robot
        self.right_trans = self.get_transformation(str(self.right_camera_source),str(self.right_camera_target))
        self.left_trans = self.get_transformation(str(self.left_camera_source),str(self.left_camera_target))

        # Initialize pointclouds
        self.pc_right, self.pc_left = None, None

        # Set up subscriber and publisher
        self.sub_right = rospy.Subscriber("/spot/depth/frontright/depth_in_visual", PointCloud2, self.pc_callback, "right", queue_size=2)
        self.sub_left = rospy.Subscriber("/spot/depth/frontleft/depth_in_visual", PointCloud2, self.pc_callback, "left", queue_size=2)
        self.pub = rospy.Publisher("/spot/depth/pc_plane_segmentation", PointCloud2)

        # Setup ROS timer and periodic callback
        self.timer = rospy.Timer(rospy.Duration(self.update_rate), self.timer_callback)

    
    def timer_callback(self, timer) -> None:
        if self.pc_right is not None and self.pc_left is not None:
            # Combine both pointclouds into one array
            points = np.vstack((self.pc_right, self.pc_left))

            # Ground plane segmentation based on Z-coordinate
            non_ground_points = points[np.where(points[:, 2] > self.segmentation_treshold)]

            # Create PointCloud2 message from array
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = str(self.left_camera_target)[1:]
            msg = point_cloud2.create_cloud_xyz32(header, non_ground_points)

            # Publish the resulting pointcloud
            self.pub.publish(msg)


    def pc_callback(self, pc_msg, args) -> None:
        # Make from the input PointCloud2 message a numpy array
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)

        # Make pointcloud homogeneous
        homogeneous_pc = np.hstack((pc, np.ones(len(pc)).reshape(-1, 1)))

        # Apply transformation and convert back to normal coordinates
        if args == "right":
            pc_transformed = np.dot(self.right_trans, homogeneous_pc.T).T
            self.pc_right = pc_transformed[:, :3] / pc_transformed[:, -1:]
        else:
            pc_transformed = np.dot(self.left_trans, homogeneous_pc.T).T
            self.pc_left = pc_transformed[:, :3] / pc_transformed[:, -1:]


    # def segment_ground(self, points: np.ndarray) -> np.ndarray:
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(points)
    #     plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    #     rospy.loginfo("Plane equation: %fx + %fy + %fz + %f = 0", plane_model[0], plane_model[1], plane_model[2], plane_model[3])

    #     inlier_cloud = pcd.select_by_index(inliers)
    #     inlier_cloud.paint_uniform_color([1.0, 0, 0])
    #     outlier_cloud = pcd.select_by_index(inliers, invert=True)

    #     return outlier_cloud
        

    def get_transformation(self, source: str, target: str, latest=False) -> np.ndarray:
        trans_matrix = np.zeros((4, 4))
        position, quaternions = None, None

        if latest:
            try:
                time = self.tf_listener.getLatestCommonTime(source, target)
                position, quaternions = self.tf_listener.lookupTransform(target, source, time)

            except tf.Exception as e:
                rospy.logwarn("Failed to lookup transform: {}".format(e))

        else:
            # Wait for transform to become available
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
        PlaneSegmentation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass




# # Global variable to store the last time a message was processed
# last_processed_time = None

# # Initialize node
# rospy.init_node('pointcloud_listener', anonymous=True)
# pose = tf.TransformListener()

# def callback(pc_msg):
#     # Global variable to keep track of the last prossed time
#     global last_processed_time

#     # Check if the last_processed_time is initialized
#     if last_processed_time is None:
#         # Initialize the last_processed_time with the current message timestamp
#         last_processed_time = pc_msg.header.stamp
#     else:
#         # Calculate the time elapsed since the last processed message
#         time_elapsed = pc_msg.header.stamp - last_processed_time

#         if time_elapsed >= rospy.Duration(1):
#             ground_threshold = 0.5

#             # Extract the number of points from the PointCloud2 message
#             pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pc_msg)
#             pc = np.dot

#             try:
#                 time = pose.getLatestCommonTime("/frontleft_depth_frame","/base_footprint")
#                 position, quaternion = pose.lookupTransform("/base_footprint", "/frontleft_depth_frame", time)

#                 rospy.loginfo("Position: %f, %f, %f", position[0], position[1], position[2])
#                 rospy.loginfo("Quaternion: %f, %f, %f, %f", quaternion[0], quaternion[1], quaternion[2], quaternion[2])
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                 rospy.loginfo("Failed")

            

#             # rospy.loginfo("Initial shape of array: (%i, %i)", pc.shape[0], pc.shape[1])
#             # rospy.loginfo("Fist point: (x: %f, y: %f, z: %f)", pc[0][0], pc[0][1], pc[0][2])
#             # rospy.loginfo("Min: (x: %f, y: %f, z: %f)", np.min(pc[:, 0]), np.min(pc[:, 1]), np.min(pc[:, 2]))
#             # rospy.loginfo("Max: (x: %f, y: %f, z: %f)", np.max(pc[:, 0]), np.max(pc[:, 1]), np.max(pc[:, 2]))

#             # filtered_pc = pc[np.where(pc[:, 2] == 0.0)]
#             # # test = np.unique(pc[:, 2])
#             # rospy.loginfo("Filtered shape of array: (%i, %i)", filtered_pc.shape[0], filtered_pc.shape[1])

#             last_processed_time = pc_msg.header.stamp



# def listener():
#     # Subscribe to the "/spot/depth/frontleft/depth_in_visual" topic
#     rospy.Subscriber("/spot/depth/frontright/depth_in_visual", PointCloud2, callback)

#     rospy.spin()

# if __name__ == '__main__':
#     listener()
