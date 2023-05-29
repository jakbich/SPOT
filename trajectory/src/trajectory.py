import rospy
import numpy as np
import tf

import actionlib
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult


class Trajectory:
    def __init__(self) -> None:
        # Initialize node
        rospy.init_node("trajectory")
        rospy.logwarn("Trajectory node initialized")

        # Setup transform listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))

        # Setup subscriber
        self.path_sub = rospy.Subscriber("/spot/planning/path", GoalStatusArray, self.path_callback)

    
    def path_callback(self, msg: GoalStatusArray):
        action_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        action_client.wait_for_server()

        for goal_status in msg.status_list:
            if goal_status.status == 1:
                rospy.logwarn("Goal status: %s", goal_status.status)
                goal = MoveBaseGoal()
                goal.target_pose = goal_status.goal.target_pose
                action_client.send_goal(goal)
                action_client.wait_for_result()
                rospy.logwarn("Goal reached")
                
            else:
                rospy.logwarn("Goal status: %s", goal_status.status)

        rospy.logwarn("Path finished")



if __name__ == "__main__":
    try:
        Trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
