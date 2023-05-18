import rospy
import ros_numpy
import numpy as np
import tf
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

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
        self.spot_size = 5

        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))

        # Set up subscriber, callback and publisher
        # -- use an updated input from the map as a callback 
        # -- to move
        self.sub_map = rospy.Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, self.callback_update)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.grid_publisher = rospy.Publisher('/spot/mapping/occupancy_grid_example', OccupancyGrid, queue_size=1)

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

        
    def callback_update(self, occupancy_grid_msg):
        time = occupancy_grid_msg.header.stamp
        position, quaternions = None, None
        try:
            position, quaternions = self.tf_listener.lookupTransform(self.map_frame, self.robot_frame, time)
            position = np.array(position)
            quaternions = np.array(quaternions)

        except tf.Exception as e:
            rospy.logwarn("Failed to lookup transform: {}".format(e))

        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            yaw = euler[-1]

        if self.occupancy_map is None:
            self.occupancy_map = OccupancyMap(self.map_size, self.map_resolution, self.map_msg.info.origin.position)

        # Translate the robot position to a position in the grid map
        relative_position = position[:2] - [self.map_msg.info.origin.position.x, self.map_msg.info.origin.position.y]
        grid_position = [int(relative_position[0]/self.map_resolution), int(relative_position[1]/self.map_resolution)]

        
        # rospy.logwarn(f"Grid position: {grid_position}, Map location: {(grid_position[1] + 1) * self.map_msg.info.width + grid_position[0]}") 
        rospy.logwarn(f"Position: {position}; Quaternions: {quaternions}")
        rospy.logwarn(f"Yaw: {yaw}")

        # For testing, check the location of Spot
        self.occupancy_map.grid = np.ones(self.map_msg.info.height * self.map_msg.info.width, dtype=int) * -1
        for x in range(grid_position[0] - self.spot_size, grid_position[0] + self.spot_size + 1):
            for y in range(grid_position[1] - self.spot_size, grid_position[0] +self.spot_size + 1):
                distance = math.sqrt((x - grid_position[0])**2 + (y - grid_position[1])**2)
                if distance <= self.spot_size:
                    self.occupancy_map.grid[(y) * self.map_msg.info.width + x] = 100

        # Compute the location in front of spot (0.75m)
        lookahead_position = relative_position + [0.75 * math.cos(yaw), 0.75 * math.sin(yaw)]
        grid_lookahead = [int(lookahead_position[0]/self.map_resolution), int(lookahead_position[1]/self.map_resolution)]

        # Compute an ellipsoid in front of Spot
        major_axis, minor_axis = 10, 4
        a, b = major_axis/2, minor_axis/2
        a_prime = a * math.cos(yaw) + b * math.sin(yaw)
        b_prime = b * math.cos(yaw) + a * math.sin(yaw)

        lookahead_points = []
        for x in range(grid_lookahead[0] - int(a_prime/self.map_resolution), grid_lookahead[0] + int(a_prime/self.map_resolution) + 1):
            for y in range(grid_lookahead[1] - int(a_prime/self.map_resolution), grid_lookahead[1] + int(a_prime/self.map_resolution) + 1):
                distance = ((x - grid_lookahead[0]) * math.cos(yaw) + (y - grid_lookahead[1]) * math.sin(yaw))**2 / a**2 + ((y - grid_lookahead[1]) * math.cos(yaw) - (x - grid_lookahead[0]) * math.sin(yaw))**2 / b**2
                if distance <= 1:
                    lookahead_points.append([x, y])
                    self.occupancy_map.grid[(y) * self.map_msg.info.width + x] = -100
        

        rospy.logwarn(f"Look a head position map: {lookahead_position}")
        rospy.logwarn(f"Look a head position grid: {grid_lookahead}")

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
        self.twist_msg.linear.x = 0.5  

        self.velocity_publisher.publish(self.twist_msg)


    def move_backward(self):
        self.twist_msg.linear.x = -0.5  

        self.velocity_publisher.publish(self.twist_msg)


    def turn_left(self):
        self.twist_msg.angular.z = -1 

        self.velocity_publisher.publish(self.twist_msg)

    def turn_right(self):
        self.twist_msg.angular.z = 1

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
    