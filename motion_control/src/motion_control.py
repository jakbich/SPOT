import rospy
import ros_numpy
import numpy as np
import tf
import math
import queue

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64MultiArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal,MoveBaseActionResult, MoveBaseFeedback, MoveBaseResult



class MotionControl:
    def __init__(self) -> None:
        rospy.init_node("motion_control_node")

        #self.server = actionlib.SimpleActionServer('motion_control', MotionAction, self.get_goal, False)
        self.server = actionlib.SimpleActionServer('motion_control', MoveBaseAction, self.get_goal, False)
        self.server.start()

        # Robot variables
        self.linear_vel = 0.5 # m/s
        self.angular_vel = 0.5 # rad/s
        self.robot_pos = None
        self.robot_yaw = None
        self.goal_pos = None
        self.goal_yaw = None

        # Define tolerance variables for moving
        self.pos_tol = 3 # grid cells
        self.angle_tol = 0.1 # rad

        # Creae a Twist message template
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

        # Setup publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def get_goal(self, goal: MoveBaseGoal):
        rospy.logwarn("Get goal")
        self.goal_pos = [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y]
        self.goal_yaw = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w], 'sxyz')[-1] 

        # if self.robot_pos is not None and self.robot_yaw is not None:
        self.move_to_goal(self.goal_pos, self.goal_yaw)

    # Function to move towards goal
    def move_to_goal(self, goal, yaw=0):
        # First rotate to goal
        rospy.logwarn("Rotating to goal")

        # Calculate angle to goal
        # angle_to_goal = math.atan2(goal[1] - self.robot_pos[1], goal[0] - self.robot_pos[0]) - self.robot_yaw
        angle_to_goal = 1000 # Initialize to a large number
        rospy.logwarn("Angle to goal: {}".format(angle_to_goal))

        # While loop to rotate to the direction of the goal
        while abs(angle_to_goal) > self.angle_tol:
            # Update angle to goal
            new_msg = rospy.wait_for_message("/spot/mapping/grid_location", Float64MultiArray)
            new_pos = new_msg.data[:2]
            new_yaw = new_msg.data[2]

            angle_to_goal = math.atan2(goal[1] - new_pos[1], goal[0] - new_pos[0]) - new_yaw

            rospy.logwarn("Rotating")
            self.twist_msg.angular.z = self.angular_vel * np.sign(angle_to_goal)
            self.cmd_vel_pub.publish(self.twist_msg)

        # Spot is inline with the direction
        self.twist_msg.angular.z = 0

        # While loop to walk to the goal
        distance_to_goal = 1000 # Initialize to a large number
        while distance_to_goal > self.pos_tol:
            # Update distance to goal
            new_msg = rospy.wait_for_message("/spot/mapping/grid_location", Float64MultiArray)
            new_pos = new_msg.data[:2]
            new_yaw = new_msg.data[2]

            distance_to_goal = math.sqrt((goal[0] - new_pos[0])**2 + (goal[1] - new_pos[1])**2)

            rospy.logwarn("Moving")
            self.twist_msg.linear.x = self.linear_vel
            self.cmd_vel_pub.publish(self.twist_msg)
        
        # Spot is at the goal
        self.twist_msg.linear.x = 0

        # Publish twist
        self.cmd_vel_pub.publish(self.twist_msg)

        # While loop to take care of the final rotation
        angle_to_orientation = self.goal_yaw - new_yaw
        rospy.logwarn("Angle to orientation: {}".format(angle_to_orientation))
        while abs(angle_to_orientation) > self.angle_tol:
            # Update angle to goal
            new_msg = rospy.wait_for_message("/spot/mapping/grid_location", Float64MultiArray)
            new_yaw = new_msg.data[2]

            angle_to_orientation = self.goal_yaw - new_yaw

            rospy.logwarn("Rotating")
            self.twist_msg.angular.z = self.angular_vel * np.sign(angle_to_orientation)
            self.cmd_vel_pub.publish(self.twist_msg)

        # Set result
        result = MoveBaseActionResult()
        result.header.stamp = rospy.Time.now()  # Set the timestamp of the result
        result.status.status = 0  # Set the status code, where 0 represents success
        result.status.text = "Goal reached"  # Set an optional status text

        rospy.loginfo("Done moving to goal")
        self.server.set_succeeded(result)

        self.goal_pos = None
        self.goal_yaw = None



if __name__ == "__main__":

    try:
        
        rospy.logwarn("Starting motion control server")
        server = MotionControl()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
         