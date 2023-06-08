#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

from human_interaction.msg import ConversationAction, ConversationGoal
from explore.msg import ExploreFrontiersAction, ExploreFrontiersGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
from yolo.msg import DetectionInfo, DetectionArray


# define state HelpNeeded
class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["object_specified", "object_not_specified"],
                             output_keys=["object_id"])
        

        self.client = actionlib.SimpleActionClient('conversation', ConversationAction)
        rospy.loginfo("Waiting for 'conversation' action server...")
        self.client.wait_for_server()
        self.goal = ConversationGoal()


    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
 
        self.goal.conv_type = "give_mission"  # Change this based on your requirements
        rospy.loginfo(f"Started conversation \"{self.goal.conv_type}\"")
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        
        if result.answer:  
            # Action completed successfully
            userdata.object_id = result.answer
            return 'object_specified'
        else:
            # Action did not complete within the timeout
            return 'object_not_specified'


class Approach_ITEM(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["goal_reached", "failed_goal_reached"],
                             input_keys=["object_id"])


        self.rrt_path_client = actionlib.SimpleActionClient('rrt_path', MoveBaseAction)
        self.rrt_path_client.wait_for_server()
        rospy.logwarn("RRT path server is up.")


    def execute(self, userdata):

                
        # Getting snapshot of the detection database
        database = rospy.wait_for_message("/spot/database", DetectionArray)

        rospy.logwarn(f"Database was recorded as {database}")


        for detection in database.detections:
            if detection.type in userdata.object_id:
                rospy.logwarn(f"Found object {detection.type} in database")
            

        self.goal = MoveBaseGoal()

        self.goal.target_pose.pose.position.x = 80
        self.goal.target_pose.pose.position.y = 80

        self.rrt_path_client.send_goal(self.goal)
        result = self.rrt_path_client.wait_for_result(rospy.Duration(60))

        
        if result:  
            # Action completed successfully
            rospy.loginfo("Goal reached")
            return 'goal_reached'
        else:
            # Action did not complete within the timeout
            return 'failed_goal_reached'
        

class Approach_PERSON(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["goal_reached", "failed_goal_reached"],
                             input_keys=["object_id"])


        self.client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        rospy.loginfo("Waiting for 'motion_control' action server...")
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        

    def execute(self, userdata):

        self.goal.target_pose.pose.position.x = 80
        self.goal.target_pose.pose.position.y = 80

        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        
        if result:  
            # Action completed successfully
            rospy.loginfo("Goal reached")
            return 'goal_reached'
        else:
            # Action did not complete within the timeout
            return 'failed_goal_reached'
        

class Human_Operation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["finished", "failed"],
                             input_keys=["object_id"])
        

    def execute(self, userdata):

        rospy.logwarn("Please pick up the item using the controller")
        rospy.logwarn("After picking up, please press enter to continue")
        # wait until enter key is pressed to continue
        input("Press Enter to continue...")

        return "finished"


class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["done", "not_done"])

        self.round_counter = 0
        self.client = actionlib.SimpleActionClient('explore', ExploreFrontiersAction)
        rospy.loginfo("Waiting for 'explore' action server...")
        self.client.wait_for_server()
        self.goal = ExploreFrontiersGoal()

    def execute(self, userdata):

        rospy.loginfo(f"Started exploring")
        for i in range(2):
            rospy.loginfo(f"Calling frontier exploration service for the {i}. time")
            self.client.send_goal(self.goal)
            self.client.wait_for_result(rospy.Duration(120))
            result = self.client.get_result()
            rospy.loginfo(result)
            rospy.loginfo("Frontier exploration service called successfully")

        return "done"
        


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["finished", "failed"])


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("MAPPING", Mapping(), 
                        transitions={"done": "CONVERSATION", 
                                    "not_done": "failed"},
                        remapping={"item_id": "item_id"})
        

        smach.StateMachine.add("CONVERSATION", Conversation(), 
                               transitions={"object_specified": "APPROACH_ITEM", 
                                            "object_not_specified": "CONVERSATION"},
                               remapping={"item_id": "item_id"})

        smach.StateMachine.add("APPROACH_ITEM", Approach_ITEM(), 
                        transitions={"goal_reached": "HUMAN_OPERATION", 
                                    "failed_goal_reached": "failed"},
                        remapping={"item_id": "item_id"})
        

        smach.StateMachine.add("HUMAN_OPERATION", Human_Operation(), 
                transitions={"finished": "APPROACH_PERSON", 
                            "failed": "failed"},
                remapping={"item_id": "item_id"})


        smach.StateMachine.add("APPROACH_PERSON", Approach_PERSON(), 
                        transitions={"goal_reached": "finished", 
                                    "failed_goal_reached": "failed"},
                        remapping={"item_id": "item_id"})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
 
