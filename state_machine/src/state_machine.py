#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib

from human_interaction.msg import ConversationAction, ConversationGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest


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


class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["goal_reached", "failed_goal_reached"],
                             input_keys=["object_id"])

        
        self.client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        rospy.loginfo("Waiting for 'motion_control' action server...")
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()


    def execute(self, userdata):
        rospy.loginfo("Executing state motion_control")
 
        # if userdata.object_id == "orange":
        #     print("Object ID: ", userdata.object_id)
        #     return 'goal_reached'
        
        # else:
        #     print("Object ID: ", userdata.object_id)
        #     return 'failed_goal_reached'
        
        # self.goal.target_pose.pose.position.x = 80
        # self.goal.target_pose.pose.position.y = 80

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
        

class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["done", "not_done"])

        self.round_counter = 0
        self.client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        rospy.loginfo("Waiting for 'motion_control' action server...")
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()


    def execute(self, userdata):
        rospy.loginfo(f"Executing state mapping for the {self.round_counter} time")

        rospy.loginfo("Waiting for service /spot/explore_frontiers...")
        rospy.wait_for_service('/spot/explore_frontiers')
        rospy.loginfo("Service /spot/explore_frontiers is available")

        

        

        for i in range(10):
                # Create the connection to the service. Remeber it's a Trigger service
            frontier_service = rospy.ServiceProxy('/spot/explore_frontiers', Trigger)

            # Create an object of type TriggerRequest. We need a TriggerRequest for a Trigger service
            # We don't need to pass any argument because it doesn't take any
            frontier_trigger = TriggerRequest()
            # Now send the request through the connection
            rospy.loginfo("Calling frontier exploration service")
            result = frontier_service(frontier_trigger)

            # Print the result given by the service called
            if result.success:
                rospy.loginfo("Frontier exploration service called successfully")
            else:
                rospy.loginfo("Frontier exploration service call failed")


        # if self.round_counter == 10:
        #     rospy.loginfo("Mapping done")
        #     return 'done'
        
        # else:
        #     self.round_counter += 1
        #     rospy.loginfo(f"Exploration no. {self.round_counter} done")
        #     return 'not_done'


if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["finished", "failed"])


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("MAPPING", Mapping(), 
                        transitions={"done": "CONVERSATION", 
                                    "not_done": "MAPPING"},
                        remapping={"item_id": "item_id"})
        

        smach.StateMachine.add("CONVERSATION", Conversation(), 
                               transitions={"object_specified": "APPROACH", 
                                            "object_not_specified": "failed"},
                               remapping={"item_id": "item_id"})

        smach.StateMachine.add("APPROACH", Approach(), 
                        transitions={"goal_reached": "finished", 
                                    "failed_goal_reached": "failed"},
                        remapping={"item_id": "item_id"})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
 
