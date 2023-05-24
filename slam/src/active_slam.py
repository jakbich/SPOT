import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer

class OccupancyMap:
    def __init__(self, size: np.ndarray, resolution: float, origin: np.ndarray) -> None:
        self.size = np.array([size[0] / resolution, size[0] / resolution], dtype=int)
        self.resolution = resolution
        self.grid = np.ones(self.size[0] * self.size[1], dtype=int) * -1
        self.origin = np.array([origin.x, origin.y])

        self.map_frame = rospy.get_param("/map_frame")


class ActiveSLAM:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('active_slam')

        self.occupancy_map = None
        self.spot_size = 6 # in grid cells
        self.lookahead_distance = 50 # in grid cells

        self.linear_speed = 0.5 # m/s
        self.angular_speed = 0.5 # rad/s

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))

        # Creata a OccupancyGrid message template
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame[1:]
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size[0] / self.map_resolution)
        self.map_msg.info.height = int(self.map_size[1] / self.map_resolution)
        self.map_msg.info.origin.position.x = -self.map_size[0] / 2
        self.map_msg.info.origin.position.y = -self.map_size[1] / 2

        # Create a Twist message template
        self.twist_msg = Twist()
        # self.twist_msg.header = Header()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        # Set up subscriber, callback and publisher
        # -- use an updated input from the map as a callback 
        # -- to move
        sub_map = Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, queue_size=5)
        sub_robot_pos = Subscriber("odom/ground_truth", Odometry, queue_size=20)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.grid_publisher = rospy.Publisher('/spot/mapping/occupancy_grid_example', OccupancyGrid, queue_size=1)

        # Synchronize the subscribers based on their timestamps
        ts = ApproximateTimeSynchronizer([sub_map, sub_robot_pos], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)       


    """
    IDEE:
    - is recht vooruit vrij: loop 1 rechtdoor; update map en herhaal
        - zo niet, draai naar link -> vrij? loop 1 rechtdoor; update en herhaal;
        - helemaal vast? -> paar stappen naar achteren

    PROBLEEM:
    - hoe tracken we de robot???
        - kallman-filter?
    - hoe laten we hem x blokjes bewegen
    """

        
    def callback(self, occupancy_grid_msg, msg_robot_):
        time = occupancy_grid_msg.header.stamp
        position = np.array([msg_robot_.pose.pose.position.x, msg_robot_.pose.pose.position.y, msg_robot_.pose.pose.position.z])
        quaternions = np.array([msg_robot_.pose.pose.orientation.x, msg_robot_.pose.pose.orientation.y, msg_robot_.pose.pose.orientation.z, msg_robot_.pose.pose.orientation.w])    
        
        rospy.logwarn(f'Position {position}')

        # position, quaternions = None, None
        # try:
        #     position, quaternions = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
        #     position = np.array(position)
        #     quaternions = np.array(quaternions)

        # except tf.Exception as e:
        #     rospy.logwarn("Failed to lookup transform: {}".format(e))

        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            yaw = euler[-1]

            if self.occupancy_map is None:
                self.occupancy_map = OccupancyMap(self.map_size, self.map_resolution, self.map_msg.info.origin.position)

            # Translate robot position to a position in the grid map
            relative_position = position[:2] - [self.map_msg.info.origin.position.x, self.map_msg.info.origin.position.y]
            grid_position = [int(relative_position[0]/self.map_resolution), int(relative_position[1]/self.map_resolution)]

            rospy.logwarn(f'Grid position: {grid_position}, Map location: {(grid_position[1] + 1) * self.map_msg.info.width + grid_position[0]}')

            # For testing, track the location of Spot in the map
            # Create a circle of occupancy around Spot
            grid_position_area = []
            self.occupancy_map.grid = np.ones(self.map_msg.info.height * self.map_msg.info.width, dtype=int) * -1
            for i in range(-self.spot_size, self.spot_size):
                for j in range(-self.spot_size, self.spot_size):
                    if i**2 + j**2 < self.spot_size**2:
                        grid_position_area.append((grid_position[1] + i) * self.map_msg.info.width + grid_position[0] + j)
                        self.occupancy_map.grid[grid_position_area[-1]] = 50

            self.occupancy_map.grid[grid_position[1] * self.map_msg.info.width + grid_position[0]] = 100

            # Shoot a ray in front of Spot to check if there is an obstacle
            direction = np.array([np.cos(yaw), np.sin(yaw)])
            start = np.array(grid_position)
            end = np.array(grid_position) + self.lookahead_distance * direction

            # Set the grid value to -100 for each block between start and end
            for i in range(int(np.linalg.norm(end - start))):
                self.occupancy_map.grid[int(start[1]) * self.map_msg.info.width + int(start[0])] = -100
                start += direction.astype(int)

            # Set the grid value to -100 at end
            # self.occupancy_map.grid[int(end[1]) * self.map_msg.info.width + int(end[0])] = -100


            # # Compute the grid position in front of Spot using the yaw
            # view_grid_position = [int(grid_position[0] + self.lookahead_distance * np.cos(yaw)), int(grid_position[1] + self.lookahead_distance * np.sin(yaw))]
            
            # # Compute circle in front of Spot to check and give it value -100 on the grid using the yaw
            # view_grid_area = []
            # for i in range(-self.lookahead_distance, self.lookahead_distance):
            #     for j in range(-self.lookahead_distance, self.lookahead_distance):
            #         if i**2 + j**2 < self.lookahead_distance**2:
            #             view_grid_area.append((view_grid_position[1] + i) * self.map_msg.info.width + view_grid_position[0] + j)
            #             self.occupancy_map.grid[view_grid_area[-1]] = -50
            
            # self.occupancy_map.grid[view_grid_position[1] * self.map_msg.info.width + view_grid_position[0]] = -100
            
            
            # major_axis, minor_axis = 10, 4
            # a, b = major_axis / 2, minor_axis / 2
            # a_prime = a * np.cos(yaw) + b * np.sin(yaw) 
            # b_prime = b * np.cos(yaw) + a * np.sin(yaw)

            # view_grid_area = []
            # for i in range(0, major_axis):
            #     for j in range(0, minor_axis):
            #         if (i/a)**2 + (j/b)**2 < 1:
            #             self.occupancy_map.grid[(view_grid_position[1] + i) * self.map_msg.info.width + view_grid_position[0] + j] = -100


        
            # for x in range(lookahead_grid_position[0] - int(a_prime/self.map_resolution), lookahead_grid_position[0] + int(a_prime/self.map_resolution) + 1):
            #     for y in range(lookahead_grid_position[1] - int(a_prime/self.map_resolution), lookahead_grid_position[1] + int(a_prime/self.map_resolution) + 1):
            #         distance = ((x - lookahead_grid_position[0]) * math.cos(yaw) + (y - lookahead_grid_position[1]) * math.sin(yaw))**2 / a**2 + ((y - lookahead_grid_position[1]) * math.cos(yaw) - (x - lookahead_grid_position[0]) * math.sin(yaw))**2 / b**2
            #         if distance <= 1:
            #             # lookahead_points.append([x, y])
            #             self.occupancy_map.grid[(y) * self.map_msg.info.width + x] = -100
            
            self.map_msg.data = self.occupancy_map.grid
            self.map_msg.header.stamp = time
            self.grid_publisher.publish(self.map_msg)


        # # Read synchronised time
        # time = occupancy_grid_msg.header.stamp

        # resolution = occupancy_grid_msg.info.resolution

        # width = occupancy_grid_msg.info.width / resolution
        # height = occupancy_grid_msg.info.height / resolution

        # pos_x = occupancy_grid_msg.info.origin.position.x / resolution
        # pos_y = occupancy_grid_msg.info.origin.position.y / resolution

        # grid = occupancy_grid_msg.data

        # # IDEE:
        # # - Soort van range; grotere blik maken op wat vrij is
        # if grid[(pos_y + 1) * width + pos_x] == 0:
        #     self.move_forward
        # else:
        #     self.turn_right


    def move_forward(self):
        self.twist_msg.linear.x = self.linear_speed  

        self.velocity_publisher.publish(self.twist_msg)


    def move_backward(self):
        self.twist_msg.linear.x = -self.linear_speed   

        self.velocity_publisher.publish(self.twist_msg)


    def turn_left(self):
        self.twist_msg.angular.z = -self.angular_speed

        self.velocity_publisher.publish(self.twist_msg)

    def turn_right(self):
        self.twist_msg.angular.z = self

        self.velocity_publisher.publish(self.twist_msg)

    def stop(self):
        self.twist_msg.linear.x = 0
        self.twist_msg.angular.z = 0

        self.velocity_publisher.publish(self.twist_msg)



        

if __name__ == '__main__':
    try:
        ActiveSLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
    