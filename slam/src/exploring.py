import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue
import actionlib 

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, Float64MultiArray
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer


class BasicExploration:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node('basic_exploration')

        # Setup transformation listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2)) # Wait for transform listener to start
        
        # Create a goal template
        self.goal_msg = Float64MultiArray()

        self.occupancy_grid = None
        self.robot_pos = None
        self.robot_yaw = None
        self.done_flag = True

        self.rotation_count = 0
        self.rotated = False

        self.rate = rospy.Rate(1)

        # Setup subscriber for occupancy grid
        self.sub_occ_grid = rospy.Subscriber("/spot/mapping/occupancy_grid", OccupancyGrid, self.grid_callback, queue_size=5)
        self.sub_pos = rospy.Subscriber("/spot/mapping/grid_location", Float64MultiArray, self.robot_pos_callback, queue_size=5)
        self.sub_done_flag = rospy.Subscriber("/spot/mapping/done", Bool, self.done_callback)

        # Setup publisher for goals
        self.pub_goal = rospy.Publisher("/spot/mapping/goal", Float64MultiArray, queue_size=5)
    
    def grid_callback(self, msg):
        rospy.logwarn("Occupancy grid callback")
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def robot_pos_callback(self, msg):
        rospy.logwarn("Robot position callback")
        self.robot_pos = msg.data[:2]
        self.robot_yaw = msg.data[2]

    def done_callback(self, msg):
        rospy.logwarn("Done callback")
        self.done_flag = msg.data


    def explore(self):
        if self.occupancy_grid is not None and self.robot_pos is not None and self.robot_yaw is not None:
            if self.done_flag:

                # Check whether the space in front of the robot is occupied by using the yaw and position
                lookahead_center = (self.robot_pos + np.array([math.cos(self.robot_yaw), math.sin(self.robot_yaw)]) * 15).astype(int)
                width = 3 # grids
                length = 5 # grids

                lookahead_area = []
                # Add grids in the width and length to the left and right of the lookahead center
                for i in range(-width, width+1):
                    for j in range(length):
                        coordinate = [lookahead_center + np.array([math.cos(self.robot_yaw + math.pi/2) * i, math.sin(self.robot_yaw + math.pi/2) * i]) + np.array([math.cos(self.robot_yaw) * j, math.sin(self.robot_yaw) * j])]
                        # rospy.logwarn(coordinate[0][1])
                        lookahead_area.append([int(coordinate[0][0]), int(coordinate[0][1])])

                # Check whether the grids in the lookahead area are occupied
                rospy.logwarn(lookahead_area)
                x = np.array(lookahead_area)[:, 0]
                y = np.array(lookahead_area)[:, 1]
        
                values = self.occupancy_grid[x, y]

                if np.any(values) >= 50:
                    # If the grids are occupied, rotate the robot 90 degrees
                    self.goal_msg.data = [self.robot_pos[0], self.robot_pos[1], self.robot_yaw + math.pi/2]
                    self.pub_goal.publish(self.goal_msg)
                    # Else move the robot forward
                    rospy.logwarn(f"Area is occupied, {self.done_flag}")
                else:
                    self.goal_msg.data = [lookahead_center[0], lookahead_center[1], self.robot_yaw]
                    self.pub_goal.publish(self.goal_msg)
                    rospy.logwarn(f"Current position: {self.robot_pos}, Goal position: {lookahead_center}")
                    rospy.logwarn(f"Area is empty, {self.done_flag}")

    def run(self):
        while not rospy.is_shutdown():
            self.explore()
            self.rate.sleep()

            

    

    
if __name__ == "__main__":
    try:
        be = BasicExploration()
        be.run()
    except rospy.ROSInterruptException:
        pass

