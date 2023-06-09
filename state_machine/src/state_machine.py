#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import time

from human_interaction.msg import ConversationAction, ConversationGoal
from explore.msg import ExploreFrontiersAction, ExploreFrontiersGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
from yolo.msg import DetectionInfo, DetectionArray
from std_msgs.msg import String


class Mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["done", "threshold_not_reached"])

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
    



class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["request_speech", "request_text", "no_request"])

    def execute(self, userdata):

        rospy.loginfo("Executing state Idle")

        rospy.sleep(5)

        return "request_speech"




# define state Conversation
class Conversation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["object_specified", "object_not_specified", "failed_3_times"],
                             output_keys=["object_id"])
        
        self.client = actionlib.SimpleActionClient('conversation', ConversationAction)
        rospy.loginfo("Waiting for 'conversation' action server...")
        self.client.wait_for_server()
        self.goal = ConversationGoal()


    def execute(self, userdata):
        rospy.loginfo("Executing state Conversation")
 
        self.pub = rospy.Subscriber("/spot/mission_status", String, queue_size=3)

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
        self.goal.target_pose.pose.orientation.z = 2

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
        # use rrt path to object/person by coding z to 2
        self.goal.target_pose.pose.orientation.z = 2

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
        

class PickItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["successfully_picked", "failed_more_than_3_times", "failed"])
        
        self.failed_counter = 0

    def execute(self, userdata):

        rospy.logwarn("Please pick up the item using the controller")
        rospy.logwarn("After picking up, please press enter to continue")
        # wait until enter key is pressed to continue, if nothing is pressed after 60 seconds then return failed
        
        success = wait_for_input(60)

        if success:
            return "successfully_picked"
        
        elif self.failed_counter < 3:
            return "failed"
        
        else:
            return "failed_more_than_3_times"
        
        
    
    
class PlaceItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=["successfully_placed", "failed_more_than_3_times", "failed"])
        self.failed_counter = 0
  
    def execute(self, userdata):

        rospy.logwarn("Please pick up the item using the controller")
        rospy.logwarn("After picking up, please press enter to continue")
        # wait until enter key is pressed to continue, if nothing is pressed after 60 seconds then return failed

        success = wait_for_input(60)

        if success:
            return "successfully_picked"
        
        elif self.failed_counter < 3:
            return "failed"
        
        else:
            return "failed_more_than_3_times"


class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failed"])

    def execute(self, userdata):

        rospy.logwarn("System is initializing")
        rospy.sleep(5)
        return "success"
        

class ConfirmMission(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=["mission_confirmed", "mission_not_confirmed"])
                                

        self.client = actionlib.SimpleActionClient('conversation', ConversationAction)
        rospy.loginfo("Waiting for 'conversation' action server...")
        self.client.wait_for_server()
        self.goal = ConversationGoal()

    def execute(self, userdata):
        rospy.loginfo("Executing state ConfirmMission")
        self.pub = rospy.Subscriber("/spot/mission_status", String, queue_size=3)

        self.goal.conv_type = "confirm_mission"  # Change this based on your requirements
        rospy.loginfo(f"Started conversation \"{self.goal.conv_type}\"")
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        
        if result.answer:  
            # Action completed successfully
            return 'mission_confirmed'
        else:
            # Action did not complete within the timeout
            return 'mission_not_confirmed'


class Estop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["triggered"])

        # set up client to "/spot/estop/gentle" service
        self.client = rospy.ServiceProxy("/spot/estop/gentle", Trigger)

    def execute(self, userdata):

        # trigger estop
        rospy.logwarn("Estop triggered")
        self.client.wait_for_service()
        self.client.call(TriggerRequest())
        return "triggered"


def wait_for_input(timeout):
    start_time = time.time()
    while input("Press enter to continue...") != '':
        if time.time() - start_time >= timeout:
            print("Timeout reached. Continuing...")
            return False
    
    else:
        print("Input received. Continuing...")
        return True



if __name__ == '__main__':
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[ "failed"])


    # Open the container
    with sm:

        # Add states to the container
        smach.StateMachine.add("INITIALIZATION", Initialization(), 
                        transitions={"success": "MAPPING", 
                                    "failed": "ESTOP"},
                        remapping={"item_id": "item_id"})
        
        # Add states to the container
        smach.StateMachine.add("ESTOP", Estop(),
                        transitions={"triggered": "failed"},
                        remapping={"item_id": "item_id"})


        # Add states to the container
        smach.StateMachine.add("MAPPING", Mapping(), 
                        transitions={"done": "IDLE", 
                                    "threshold_not_reached": "MAPPING"},
                        remapping={"item_id": "item_id"})
        

                # Add states to the container
        smach.StateMachine.add("IDLE", Idle(), 
                        transitions={"request_speech": "CONVERSATION", 
                                    "request_text": "APPROACH_ITEM",
                                    "no_request": "IDLE"},
                        remapping={"item_id": "item_id"})


        smach.StateMachine.add("CONVERSATION", Conversation(), 
                               transitions={"object_specified": "APPROACH_ITEM", 
                                            "object_not_specified": "CONVERSATION",
                                            "failed_3_times": "ESTOP"},
                               remapping={"item_id": "item_id"})

        smach.StateMachine.add("APPROACH_ITEM", Approach_ITEM(), 
                        transitions={"goal_reached": "PICK_ITEM", 
                                    "failed_goal_reached": "ESTOP"},
                        remapping={"item_id": "item_id"})


        smach.StateMachine.add("PICK_ITEM", PickItem(), 
                transitions={"successfully_picked": "APPROACH_PERSON",
                            "failed": "PICK_ITEM", 
                            "failed_more_than_3_times": "ESTOP"},
                remapping={"item_id": "item_id"})
        

        smach.StateMachine.add("APPROACH_PERSON", Approach_PERSON(), 
                        transitions={"goal_reached": "CONFIRM_MISSION", 
                                    "failed_goal_reached": "ESTOP"},
                        remapping={"item_id": "item_id"})

        smach.StateMachine.add("CONFIRM_MISSION", ConfirmMission(), 
                transitions={"mission_confirmed": "PLACE_ITEM",
                            "mission_not_confirmed": "APPROACH_ITEM"},
                remapping={"item_id": "item_id"})

        smach.StateMachine.add("PLACE_ITEM", PlaceItem(), 
                transitions={"successfully_placed": "IDLE",
                            "failed": "PLACE_ITEM", 
                            "failed_more_than_3_times": "ESTOP"},
                remapping={"item_id": "item_id"})
        

    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SYSTEM_LAUNCH')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
 
