import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float64, Bool
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer

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
        occupancy_grid_sub = Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, queue_size=5)
        robot_pos_sub = Subscriber("/spot/ground_truth", Odometry, queue_size=20)

        # Synchronize occupancy grid and odometry
        ts = ApproximateTimeSynchronizer([occupancy_grid_sub, robot_pos_sub], queue_size=5, slop=0.1)
        ts.registerCallback(self.callback)

        # Setup publisher
        # Publisher for grid location and yaw
        self.grid_location_pub = rospy.Publisher("/spot/mapping/grid_location", Point, queue_size=5)
        self.yaw_pub = rospy.Publisher("/spot/mapping/yaw", Float64, queue_size=5)

        # Publisher for goals
        self.goal_pub = rospy.Publisher("/spot/mapping/goal", Point, queue_size=5)
        self.yaw_goal_pub = rospy.Publisher("/spot/mapping/yaw_goal", Float64, queue_size=5)    

        # Subscriber for done flag
        self.done_sub = rospy.Subscriber("/spot/mapping/done", Bool, self.done_callback)

        # Start exploration loop
        # rospy.Timer(rospy.Duration(1), self.explore_frontiers) # 1 Hz

        rospy.sleep(rospy.Duration(2))
        self.move_to_position([5,5])


    # callback function for occupancy grid and odometry updates
    def callback(self, occupancy_grid_msg, robot_pos_msg):
        self.occupancy_grid = np.array(occupancy_grid_msg.data).reshape((occupancy_grid_msg.info.height, occupancy_grid_msg.info.width))
        position = np.array([robot_pos_msg.pose.pose.position.x, robot_pos_msg.pose.pose.position.y])
        quaternions = np.array([robot_pos_msg.pose.pose.orientation.x, robot_pos_msg.pose.pose.orientation.y, robot_pos_msg.pose.pose.orientation.z, robot_pos_msg.pose.pose.orientation.w])

        if position is not None and quaternions is not None:
            euler = tf.transformations.euler_from_quaternion(quaternions, 'sxyz')
            self.yaw = euler[-1]

            relative_position = position - [occupancy_grid_msg.info.origin.position.x, occupancy_grid_msg.info.origin.position.y]
            self.robot_pos = np.array([int(relative_position[0]/occupancy_grid_msg.info.resolution), int(relative_position[1]/occupancy_grid_msg.info.resolution)])

    # callback function for done flag
    def done_callback(self, msg):
        self.done_flag = msg.data

    
    # exploration loop function
    def explore_frontiers(self):
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
                if self.occupancy_grid[pos[0], pos[1]] == -1:
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            if is_valid_position([pos[0] + i, pos[1] + j]) and self.occupancy_grid[pos[0] + i, pos[1] + j] == 0:
                                return True
                return False
            
            frontier_queue.put((0, self.robot_pos))

            # Initialize visited set
            visited = set()
            visited.add(tuple(self.robot_pos))

            # Start exploration loop
            while not frontier_queue.empty():
                # Get next frontier
                _, pos = frontier_queue.get()

                # Check if frontier is valid
                if is_valid_position(pos) and is_frontier(pos):
                    # Publish to motion controller
                    self.goal_pub.publish(Point(pos[0], pos[1], 0))
                    self.yaw_goal_pub.publish(Float64(0))
                    self.grid_location_pub.publish(Point(self.robot_pos[0], self.robot_pos[1], 0))
                    self.yaw_pub.publish(Float64(self.yaw))

                    # Wait for robot to reach goal
                    while not rospy.is_shutdown() and not self.done_flag:
                        # Check if robot is at goal
                        if self.done_flag:
                            break

                        rospy.spinOnce()
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

          
            rospy.loginfo("Exploration complete")



if __name__ == '__main__':
    try:
        FrontierExploration()
        rospy.spin()
        rospy.sleep(rospy.Duration(10))
    except rospy.ROSInterruptException:
        rospy.logwarn("The node plane_segmentation could not be launch")
        pass
    