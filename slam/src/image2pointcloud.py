#!/usr/bin/env python

import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, PointField
import numpy as np
from message_filters import TimeSynchronizer, Subscriber

class CreatePointClouds:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node("CreatePointClouds")

        self.fx = 319.9988245765257  # Focal length in x-direction
        self.fy = 319.9988245765257  # Focal length in y-direction
        self.cx = 320.5  # Principal point x-coordinate
        self.cy = 240.5  # Principal point y-coordinate

        # Set up subscriber and publisher
        sub_right_image = Subscriber("/spot/depth/frontright/image", Image, queue_size=1)
        sub_left_image = Subscriber("/spot/depth/frontleft/image", Image, queue_size=1)

        self.pub_right = rospy.Publisher("/spot/depth/right/pointcloud", PointCloud2, queue_size=2)
        self.pub_left = rospy.Publisher("/spot/depth/left/pointcloud", PointCloud2, queue_size=2)

        ts = TimeSynchronizer([sub_right_image, sub_left_image], queue_size=2) # Slop is time tollerance
        ts.registerCallback(self.callback)

    def callback(self, right_image, left_image):
        rospy.logwarn("Pointcloud")
        right_pointcloud_msg = self.image2pointcloud_msg(right_image)
        left_pointcloud_msg = self.image2pointcloud_msg(left_image)

        self.pub_right.publish(right_pointcloud_msg)
        self.pub_left.publish(left_pointcloud_msg)

    def image2pointcloud_msg(self, image_msg):
        # Convert depth image message to a NumPy array
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')

        # Create an empty point cloud message
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header = image_msg.header
        point_cloud_msg.height = depth_image.shape[0]
        point_cloud_msg.width = depth_image.shape[1]
        point_cloud_msg.fields.append(PointField(
            name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        point_cloud_msg.fields.append(PointField(
            name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        point_cloud_msg.fields.append(PointField(
            name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        point_cloud_msg.point_step = 12
        point_cloud_msg.row_step = (
            point_cloud_msg.point_step * point_cloud_msg.width)
        point_cloud_msg.is_dense = True
        point_cloud_msg.is_bigendian = False

        # Reshape the depth image for vectorized operations
        rows, cols = depth_image.shape
        depth_flat = depth_image.reshape(-1)

        # Compute pixel coordinates
        u = np.arange(cols)
        v = np.arange(rows)
        u, v = np.meshgrid(u, v)
        u = u.reshape(-1)
        v = v.reshape(-1)

        # Compute 3D coordinates
        x = (u - self.cx) * depth_flat / self.fx
        y = (v - self.cy) * depth_flat / self.fy
        z = depth_flat

        # Filter out invalid depth values
        valid_indices = np.logical_and(
            np.logical_and(depth_flat > 0, ~np.isnan(depth_flat)),
            ~np.isinf(depth_flat)
        )
        x = x[valid_indices]
        y = y[valid_indices]
        z = z[valid_indices]

        # Create a point cloud message
        return create_cloud_xyz32(image_msg.header, np.stack((x, y, z), axis=-1))

if __name__ == '__main__':
    try:
        CreatePointClouds()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
