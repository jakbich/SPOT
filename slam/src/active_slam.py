import rospy
import ros_numpy
import numpy as np
import tf

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid

def create__example_map():
    # Initialize the grid message
    grid_msg = OccupancyGrid()
    grid_msg.header.frame_id = "map"
    grid_msg.info.resolution = 0.1  # 10cm per pixel
    grid_msg.info.width = 30
    grid_msg.info.height = 30
    grid_msg.info.origin.position.x = -1.5  # Bottom left corner of the map
    grid_msg.info.origin.position.y = -1.5
    grid_msg.info.origin.orientation.w = 1.0  # No rotation

    # Initialize the data array with -1 (unknown)
    grid_msg.data = [-1] * grid_msg.info.width * grid_msg.info.height

    # Set the borders to occupied (100)
    for x in range(grid_msg.info.width):
        grid_msg.data[x] = 100
        grid_msg.data[x + (grid_msg.info.height - 1) * grid_msg.info.width] = 100
    for y in range(grid_msg.info.height):
        grid_msg.data[y * grid_msg.info.width] = 100
        grid_msg.data[(y + 1) * grid_msg.info.width - 1] = 100

    # Set the starting point to free (0)
    start_x = 10
    start_y = 15
    grid_msg.data[start_y * grid_msg.info.width + start_x] = 0

    return grid_msg


class ActiveSLAM:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('active_slam')

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))

        # Get parameters from server
        
        # --idea
        self.grid_map = rospy.get_param("/slam/grid_map")

        # Set up subscriber, callback and publisher
        # -- use an updated input from the map as a callback 
        # -- to move
        self.sub_map = rospy.Subscriber("/spot/grid_map", Int32MultiArray, callback_update)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Set up start values
        self.start_pos = [0, 0]
        self.curren_pos = self.start_pos
        self.direction = 'forward' # left, right, backward

    """
    IDEE:
    - is recht vooruit vrij: loop 1 rechtdoor; update map en herhaal
        - zo niet, draai naar link -> vrij? loop 1 rechtdoor; update en herhaal;
        - helemaal vast? -> paar stappen naar achteren
    """

        
    def callback_update(self, data: OccupancyGrid):
        pass


    def move_forward(self):
        twist = Twist()


    def move_backward(self):
        pass

    def move_left(self):
        pass

    def move_right(self):
        pass



        

if __name__ == 'main':
    try:
        ActiveSLAM()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logwarn("The node active_slam could not be launched")
        pass
    