import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

class MotionControl:
    def __init__(self) -> None:
        rospy.init_node('motion_control')

        # Robot variables
        self.linear_vel = 0.5 # m/s
        self.angular_vel = 1 # rad/s
        self.robot_pos = None
        self.robot_yaw = None
        self.goal_pos = None
        self.goal_yaw = None

        # Define tolerance variables for moving
        self.pos_tol = 10 # grid cells
        self.angle_tol = 0.1 # rad

        # Creae a Twist message template
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        # Setup subscriber
        self.goal_sub = rospy.Subscriber("/spot/mapping/goal", Float64MultiArray, self.goal_callback)
        self.robot_pos_sub = rospy.Subscriber("/spot/mapping/grid_location", Float64MultiArray, self.robot_pos_callback, queue_size=1)

        # Setup publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.done_pub = rospy.Publisher("/spot/mapping/done", Bool, queue_size=1)


    # Callback for position
    def robot_pos_callback(self, msg: Float64MultiArray):
        rospy.logwarn("Robot position callback")
        self.robot_pos = msg.data[:2]
        self.robot_yaw = msg.data[2]

        # if self.goal_pos is not None and self.goal_yaw is not None:
        #     self.move_to_goal(self.goal_pos, self.goal_yaw)

        # self.move_to_goal([95,105])
        self.move_to_goal([60,85])

    
    # Callback for goal
    def goal_callback(self, msg: Float64MultiArray):
        rospy.logwarn("Goal callback")
        self.goal_pos = msg.data[:2]
        self.goal_yaw = msg.data[2]


    # Function to move towards goal
    def move_to_goal(self, goal, yaw=0):
        # First rotate to goal
        rospy.logwarn("Rotating to goal")

        # Calculate angle to goal
        angle_to_goal = math.atan2(goal[1] - self.robot_pos[1], goal[0] - self.robot_pos[0]) - self.robot_yaw
        rospy.logwarn("Angle to goal: {}".format(angle_to_goal))

        if abs(angle_to_goal) > self.angle_tol:
            rospy.logwarn("Rotating")
            self.twist_msg.angular.z = self.angular_vel * np.sign(angle_to_goal)
            rospy.logwarn("Angular velocity: {}".format(self.twist_msg.angular.z))
            self.cmd_vel_pub.publish(self.twist_msg)
        
        else:
            rospy.logwarn("Done rotating")
            self.twist_msg.angular.z = 0

            # Move towards goal
            rospy.logwarn("Moving towards goal")
            distance_to_goal = math.sqrt((goal[0] - self.robot_pos[0])**2 + (goal[1] - self.robot_pos[1])**2)
            rospy.logwarn("Distance to goal: {}".format(distance_to_goal))

            if distance_to_goal > self.pos_tol:
                rospy.logwarn("Moving")
                self.twist_msg.linear.x = self.linear_vel
                rospy.logwarn("Linear velocity: {}".format(self.twist_msg.linear.x))
                self.cmd_vel_pub.publish(self.twist_msg)
            else:
                rospy.logwarn("Done moving")
                self.twist_msg.linear.x = 0
                self.cmd_vel_pub.publish(self.twist_msg)
                self.done_pub.publish(True)

                # Reset goal
                self.goal_pos = None
                self.goal_yaw = None


if __name__ == "__main__":
    try:
        mc = MotionControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         