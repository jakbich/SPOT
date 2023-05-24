import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float64MultiArray
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer

class OccupancyMap:
    def __init__(self, size: np.ndarray, resolution: float, origin: np.ndarray) -> None:
        self.size = np.array([size[0] / resolution, size[0] / resolution], dtype=int)
        self.resolution = resolution
        self.grid = np.ones(self.size[0] * self.size[1], dtype=int) * -1
        self.origin = np.array([origin.x, origin.y])

        self.map_frame = rospy.get_param("/map_frame")

class FrontierExploration:
    def __init__(self) -> None:
        rospy.init_node('frontier_exploration')

        # Initialize occupancy grid
        self.occupancy_grid = None

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))

        # Define robot variables
        self.robot_pos = None
        self.yaw = None
        self.linear_vel = 0.5 # m/s
        self.angular_vel = 0.5 # rad/s
        self.done_flag = False # Flag to indicate whether the robot is done moving

        # Define tolerance variables for moving
        self.pos_tol = 5 # grid cells
        self.angle_tol = 0.1 # rad

        # Subscribe to occupancy grid and odometry
        occupancy_grid_sub = rospy.Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, self.grid_callback, queue_size=5)
        robot_pos_sub = rospy.Subscriber("/spot/mapping/grid_location", Float64MultiArray, self.robot_pos_callback, queue_size=5)

        # Synchronize occupancy grid and odometry
        # ts = ApproximateTimeSynchronizer([occupancy_grid_sub, robot_pos_sub], queue_size=5, slop=0.1)
        # ts.registerCallback(self.callback)

        # Create a goal template
        self.goal_msg = Float64MultiArray()

        # Publisher for goals
        self.goal_pub = rospy.Publisher("/spot/mapping/goal", Float64MultiArray, queue_size=5) 

        # Subscriber for done flag
        self.done_sub = rospy.Subscriber("/spot/mapping/done", Bool, self.done_callback)

        # Start exploration loop
        rospy.Timer(rospy.Duration(1), self.explore_frontiers) # 1 Hz

        # FOR DEBUGGING
        # Get parameters from server
        self.map_size = np.array([rospy.get_param("/map_size/x"), rospy.get_param("/map_size/y")])
        self.map_resolution = rospy.get_param("/map_resolution")
        self.map_frame = rospy.get_param("/map_frame")
        self.robot_frame = rospy.get_param("/map_reference_frame")

        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = self.map_frame[1:]
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = int(self.map_size[0] / self.map_resolution)
        self.map_msg.info.height = int(self.map_size[1] / self.map_resolution)
        self.map_msg.info.origin.position.x = -self.map_size[0] / 2
        self.map_msg.info.origin.position.y = -self.map_size[1] / 2

        self.map_grid = None

        self.grid_publisher = rospy.Publisher('/spot/mapping/occupancy_grid_example', OccupancyGrid, queue_size=1)
        # END FOR DEBUGGING


    # callback function for occupancy grid 
    def grid_callback(self, msg):
        rospy.logwarn("Occupancy grid callback")
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    # callback function for odometry
    def robot_pos_callback(self, msg): 
        rospy.logwarn("Robot position callback")
        self.robot_pos = msg.data[:2]
        self.yaw = msg.data[2]


    # def callback(self, occupancy_grid_msg, robot_pos_msg):
    #     self.occupancy_grid = np.array(occupancy_grid_msg.data).reshape((occupancy_grid_msg.info.height, occupancy_grid_msg.info.width))
    #     self.robot_pos = robot_pos_msg.data[:2]
    #     self.yaw = robot_pos_msg.data[2]

    #     rospy.logwarn("Occupancy grid and odometry updated")

    # callback function for done flag
    def done_callback(self, msg):
        rospy.logwarn("Done callback")
        self.done_flag = msg.data


    # exploration loop function
    def explore_frontiers(self, argument):\
        # FOR DEBUGGING
        if self.map_grid is None:
            self.map_grid = OccupancyMap(self.map_size, self.map_resolution, self.map_msg.info.origin.position)


        rospy.logwarn("Exploration started")
        if self.occupancy_grid is not None and self.robot_pos is not None and self.yaw is not None:
            frontier_queue = queue.PriorityQueue()
            height, width = self.occupancy_grid.shape

            # Define function to check wheter a position is valid
            def is_valid_position(pos):
                return pos[0] >= 0 and pos[0] < height and pos[1] >= 0 and pos[1] < width
            
            # Define function to calculate distance
            def distance(pos1, pos2):
                return np.linalg.norm(np.array(pos1) - np.array(pos2))
            
            # Define function to check whether a position is a frontier
            def is_frontier(pos):
                if self.occupancy_grid[int(pos[0]), int(pos[1])] == -1:
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            if is_valid_position([pos[0] + i, pos[1] + j]) and self.occupancy_grid[int(pos[0]) + i, int(pos[1]) + j] == 0:
                                return True
                return False
            
            frontier_queue.put((0, self.robot_pos))

            # Initialize visited set
            visited = set()
            visited.add(tuple(self.robot_pos))

            # Start exploration loop
            rospy.logwarn("Exploration loop started")
            while not frontier_queue.empty():
                # Get next frontier
                _, pos = frontier_queue.get()

                rospy.logwarn("Next frontier: " + str(pos))

                # Check if frontier is valid
                if is_valid_position(pos) and is_frontier(pos):
                    # FOR DEBUGGING
                    self.map_grid.grid[pos[0] * self.map_msg.info.width + pos[1]] = 100
                    self.map_msg.data = self.map_grid.grid
                    self.map_msg.header.stamp = rospy.Time.now()
                    self.grid_publisher.publish(self.map_msg)
                    # END FOR DEBUGGING

                    # Publish to motion controller
                    self.goal_msg.data = [int(pos[0]), int(pos[1]), 0]
                    rospy.logwarn(self.goal_msg)
                    self.goal_pub.publish(self.goal_msg)

                    # self.yaw_goal_pub.publish(Float64(0))
                    # self.grid_location_pub.publish(Point(self.robot_pos[0], self.robot_pos[1], 0))
                    # self.yaw_pub.publish(Float64(self.yaw))

                    # Wait for robot to reach goal
                    while not rospy.is_shutdown() and not self.done_flag:
                        # Check if robot is at goal
                        if self.done_flag:
                            break

                        # rospy.spinOnce()
                        rospy.sleep(rospy.Duration(0.5))

                    # Check if frontier is still valid
                    if is_frontier(pos):
                        # Add frontier to visited set
                        visited.add(tuple(pos))

                        # Add neighbouring positions to frontier queue
                        for i in range(-1, 2):
                            for j in range(-1, 2):
                                if is_valid_position([pos[0] + i, pos[1] + j]) and tuple([pos[0] + i, pos[1] + j]) not in visited:
                                    frontier_queue.put((distance([pos[0] + i, pos[1] + j], self.robot_pos), [pos[0] + i, pos[1] + j]))
                
                # Check if queue is empty
                if frontier_queue.empty():
                    # Add unvisited positions to frontier queue
                    for i in range(height):
                        for j in range(width):
                            if is_valid_position([i, j]) and tuple([i, j]) not in visited:
                                frontier_queue.put((distance([i, j], self.robot_pos), [i, j]))

          
            rospy.logwarn("Exploration complete")



if __name__ == '__main__':
    try:
        FrontierExploration()
        rospy.spin()
        # rospy.sleep(rospy.Duration(10))
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
    