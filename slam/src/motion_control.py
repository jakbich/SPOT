import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal MoveBaseFeedback, MoveBaseResult
from slam.msg import MotionAction, MotionGoal, MotionFeedback, MotionResult


class MotionControl:
    def __init__(self) -> None:

        self.server = actionlib.SimpleActionServer('motion_control', MotionAction, self.get_goal, False)
        self.server.start()

        # Robot variables
        self.linear_vel = 0.5 # m/s
        self.angular_vel = 1 # rad/s
        self.robot_pos = None
        self.robot_yaw = None
        self.goal_pos = None
        self.goal_yaw = None

        # Define tolerance variables for moving
        self.pos_tol = 5 # grid cells
        self.angle_tol = 0.1 # rad

        # Creae a Twist message template
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        # Create a Bool message template
        self.done_msg = Bool()
        self.done_msg.data = False

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
        # self.move_to_goal([60,85])

        # if self.robot_pos is not None and self.robot_yaw is not None:
        #     if self.goal_pos is not None and self.goal_yaw is not None:
        #         self.move_to_goal(self.goal_pos, self.goal_yaw)

    
    # Callback for goal
    def goal_callback(self, msg: Float64MultiArray):
        rospy.logwarn("Goal callback")
        self.goal_pos = msg.data[:2]
        self.goal_yaw = msg.data[2]

        self.done_msg.data = False

    def get_goal(self, goal: MoveBaseGoal):
        rospy.logwarn("Get goal")
        self.goal_pos = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
        self.goal_yaw = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w], 'sxyz')[-1] 

        if self.robot_pos is not None and self.robot_yaw is not None:
            self.move_to_goal(self.goal_pos, self.goal_yaw)

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
                self.cmd_vel_pub.publish(self.twist_msg)
            else:
                rospy.logwarn("Done moving")
                self.twist_msg.linear.x = 0           

                # Rotate to given yaw
                rospy.logwarn("Rotating to given yaw")
                angle_to_yaw = yaw - self.robot_yaw
                rospy.logwarn("Angle to yaw: {}".format(angle_to_yaw))

                if abs(angle_to_yaw) > self.angle_tol:
                    rospy.logwarn("Rotating")
                    self.twist_msg.angular.z = self.angular_vel * np.sign(angle_to_yaw)
                    self.twist_msg.linear.x = 0
                    self.cmd_vel_pub.publish(self.twist_msg)
                else:
                    # Yaw and goal is reached
                    rospy.logwarn("Done rotating")
                    self.twist_msg.angular.z = 0
                    self.twist_msg.linear.x = 0
                    self.cmd_vel_pub.publish(self.twist_msg)

                    # Publish done message
                    self.done_msg.data = True
                    self.done_pub.publish(self.done_msg)

                    # Defining ROS result
                    result = MotionResult()
                    result.status = 1   # 1 = success, 0 = not done, -1 = failed

                    self.server.set_succeeded(result)
                    


                # Reset goal
                self.goal_pos = None
                self.goal_yaw = None



if __name__ == "__main__":

    try:
        rospy.init_node("motion_control")
        server = MotionControl()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         