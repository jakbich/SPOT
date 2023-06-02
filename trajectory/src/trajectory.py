import rospy
import numpy as np
import tf

import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult, MoveBaseActionResult


class Trajectory:
    def __init__(self) -> None:
        rospy.init_node('trajectory_node')
        # Initialize server
        self.server = actionlib.SimpleActionServer('trajectory', GoalStatusArray, self.path_callback, False)
        self.server.start()

        self.action_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        self.action_client.wait_for_server()

        # Setup transform listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(rospy.Duration(2))


    
    def path_callback(self, msg: GoalStatusArray):

        if msg.status_list is not None:
            for goal_status in msg.status_list:
                if goal_status.status == 1:
                    rospy.logwarn("Goal status: %s", goal_status.status)
                    goal = MoveBaseGoal()
                    goal.target_pose = goal_status.goal.target_pose
                    
                    self.action_client.send_goal(goal)
                    self.action_client.wait_for_result()
                    
                    result = self.action_client.get_result()
                    if result:
                        rospy.logwarn("Goal reached")
                    else:
                        rospy.logwarn("Goal not reached")
                    
                else:
                    rospy.logwarn("Goal status: %s", goal_status.status)

            rospy.logwarn("Path finished")

            # Return succes
            result = MoveBaseActionResult()
            result.header.stamp = rospy.Time.now()
            result.status.status = 0 # Set the status code, where 0 represents success
            result.status.text = "Path finished"

            final = GoalStatusArray()
            final.status_list.append(result)

            self.server.set_succeeded(final)
        
        else:
            rospy.logwarn("No path received")



if __name__ == "__main__":
    try:
        Trajectory()
        rospy.logwarn("Starting trajectory server")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
