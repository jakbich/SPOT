import rospy
import ros_numpy
import numpy as np
import tf

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Float64MultiArray


class GridPositionTransform:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('grid_position_transform')

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Give the tf server some time to start up

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")

        # self.map_size = np.array([15, 15])
        # self.map_resolution = 0.1

        self.origin_x = -self.map_size[0]/2
        self.origin_y = -self.map_size[1]/2
        self.width = int(self.map_size[0]/self.map_resolution)
        self.height = int(self.map_size[1]/self.map_resolution)

        # Create Point message template
        self.grid_location_msg = Float64MultiArray()

        # Setup subscriber
        self.robot_pos_sub = rospy.Subscriber("/odom/ground_truth", Odometry, self.callback, queue_size=5)

        # Setup publisher
        self.grid_location_pub = rospy.Publisher("/spot/mapping/grid_location", Float64MultiArray, queue_size=1)


    def callback(self, msg):
        time = msg.header.stamp

        # Get robot position
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        quaternions = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            yaw = euler[-1]

            relative_position = position - np.array([self.origin_x, self.origin_y])
            grid_location = [int(relative_position[0]/self.map_resolution), int(relative_position[1]/self.map_resolution)]

            # Publish grid location
            self.grid_location_msg.data = [grid_location[0], grid_location[1], yaw]

            rospy.logwarn("Grid location: " + str(grid_location))
            rospy.logwarn("Yaw: " + str(yaw))

            self.grid_location_pub.publish(self.grid_location_msg)

if __name__ == '__main__':
    try:
        GridPositionTransform()
        # rospy.logwarn("Grid position transform node started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception")
        pass