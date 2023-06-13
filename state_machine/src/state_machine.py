#!/usr/bin/env python

import rospy
import smach
import smach_ros
import actionlib
import time
import numpy as np
import os
import rosnode
import argparse

from human_interaction.msg import ConversationAction, ConversationGoal, ConversationFeedback, ConversationActionGoal
from explore.msg import ExploreFrontiersAction, ExploreFrontiersGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerRequest
from yolo.msg import DetectionInfo, DetectionArray
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


def check_node_running(node_name):
    """
    Helper function that checks if a node is running
    """
    node_list = rosnode.get_node_names()
    if node_name not in node_list:
        rospy.loginfo(f"Node '{node_name}' is not running.")
        return False
    else:
        rospy.loginfo(f"Node '{node_name}' is running")
        return True


def wait_for_input(timeout):
    """
    Helper function that waits for a user input for a specified timeout duration
    """
    start_time = time.time()
    while input("Press enter to continue...") != '':
        if time.time() - start_time >= timeout:
            rospy.loginfo("Timeout reached. Continuing...")
            return False
    
    else:
        rospy.loginfo("Input received. Continuing...")
        return True
    

class Initialization(smach.State):
    """
    This state ensures that all necessary servers are up and running 
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=["success", "failed"])

        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

    def execute(self, userdata):
        
        rospy.logwarn("System is initializing...")

        # Publish the mission status
        self.mission_pub.publish("Initialization")

        rospy.sleep(3)

        # Check if node plane segmentation was launchd
        plane_seg_running = check_node_running("/plane_segmentation")

        # Check if node yolo is running
        yolo_running = check_node_running("/yolo_detection")

        # Check if node occ_map is running
        occ_map_running = check_node_running("/occupancy_map")

        # Check if node grid_transform is running
        grid_transform_running = check_node_running("/grid_position_transform")

        # Set timeout duration
        timeout_duration = rospy.Duration(10)

        # Initialize the clients for the action server explore
        self.explore_client = actionlib.SimpleActionClient('explore', ExploreFrontiersAction)
        rospy.loginfo("Waiting for 'explore' action server...")
        explore_running = self.explore_client.wait_for_server(timeout_duration)
        
        # Initialize the clients for the action server conversation
        self.conversation_client = actionlib.SimpleActionClient('conversation', ConversationAction)
        rospy.loginfo("Waiting for 'conversation' action server...")
        conv_running = self.conversation_client.wait_for_server(timeout_duration)
        
        # Initialize the clients for the action server rrt_path
        self.rrt_path_client = actionlib.SimpleActionClient('rrt_path', MoveBaseAction)
        rospy.loginfo("Waiting for 'rrt_path' action server...")
        rrt_running = self.rrt_path_client.wait_for_server(timeout_duration)

        # Initialize the clients for the action server motion_control
        self.motion_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        rospy.loginfo("Waiting for 'motion_control' action server...")
        motion_running = self.motion_client.wait_for_server(timeout_duration)

        rospy.sleep(3)

        #  Check if all action servers are running
        if not (explore_running and conv_running and rrt_running and motion_running):
            rospy.logwarn("One or more action servers are not running. Failed.")
            return "failed"
        
        # Check if all nodes are running
        elif not (plane_seg_running and yolo_running and occ_map_running and grid_transform_running):
            rospy.logwarn("One or more nodes are not running. Failed.")
            return "failed"
        
        # Everything is running
        else:
            return "success"


class Mapping(smach.State):
    """
    This state calls the frontier exploration service and based on a threshold 
    waits for it to finish.
    """

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["done", "threshold_not_reached"])

        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        # Initialize the client for the frontier exploration action server
        self.client = actionlib.SimpleActionClient('explore', ExploreFrontiersAction)
        self.goal = ExploreFrontiersGoal()

        # Initialize the counter and the threshold
        self.counter = 1
        self.map_threshold = 50
        self.map_change = 100000


    def execute(self, userdata):
        
        # # Publish the mission status
        self.mission_pub.publish("Mapping")

        if not DEBUG:
            while self.map_change > self.map_threshold:
                # Save the previous map
                prev_map  = rospy.wait_for_message("/spot/mapping/occupancy_grid", OccupancyGrid) 
                rospy.logwarn(f"Received previous map")

               # Call the frontier exploration service
                rospy.logwarn(f"Calling frontier exploration service for the {self.counter}. time")
                self.client.send_goal(self.goal)
                self.client.wait_for_result(rospy.Duration(120))
                result = self.client.get_result()
                rospy.logwarn("Frontier exploration service called successfully")

                # Save the current map
                current_map = rospy.wait_for_message("/spot/mapping/occupancy_grid", OccupancyGrid)
                rospy.logwarn(f"Received current map")
                
                # Calculate the number of changed pixels
                change_pixels = 0
                for i in range(len(current_map.data)):
                    if current_map.data[i] != prev_map.data[i]:
                        change_pixels += 1

                self.map_change = change_pixels
                self.counter += 1

                # Check if the threshold is reached
                if self.map_change > 50:
                    rospy.logwarn(f"Number of changed pixels: {self.map_change}, MAPPING called again")
                    return "threshold_not_reached"
                
                else:
                    rospy.logwarn(f"Number of changed pixels: {self.map_change}, MAPPING finished")
                    return "done"
            
        else:
            # For debugging you can only execute mapping three times instead of until the threshold is reached
            for i in range(3):
                # Call the frontier exploration service
                rospy.logwarn(f"Calling frontier exploration service for the {self.counter}. time")
                self.client.send_goal(self.goal)
                self.client.wait_for_result(rospy.Duration(120))
                result = self.client.get_result()
                rospy.logwarn("Frontier exploration service called successfully")
                self.counter += 1

            return "done"
        

class Idle(smach.State):
    """ 
    This state checks if there is a request for a mission and if there is, it
    calls the conversation service to get the mission.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["request_speech", "request_text", "no_request"])
        
        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)
        
        # set up subscriber for conv_goal
        self.conv_subscriber = rospy.Subscriber("/conversation/goal", ConversationActionGoal, self.conv_callback, queue_size=3)
        self.conv_value = None  # Variable to store the value received in conv_callback

        # set up subscriber for text_goal
        self.text_subscriber = rospy.Subscriber("/spot/text_request", String, self.text_callback, queue_size=3)
        self.text_value = None  # Variable to store the value received in text_callback


    def conv_callback(self, msg):
        """
        Callback function for the conversation goal
        """
        self.conv_value = "set"
    

    def text_callback(self, msg):
        """
        Callback function for the text goal
        """
        self.text_value = "set"


    def execute(self, userdata):

        rospy.logwarn("Executing state Idle")

        # Publish the mission status
        self.mission_pub.publish("Idle")

        rospy.sleep(5)
    
        # Check if there is a request
        if self.conv_value:
            return "request_speech"
        elif self.text_value:
            return "request_text"
        else:
            return "no_request"


# define state Conversation
class Conversation(smach.State):
    """
    This state calls the conversation service to get the mission.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["object_specified", "object_not_specified", "failed_3_times"],
                             output_keys=["object_id"])
        
        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        # Initialize the client for the conversation action server
        self.client = actionlib.SimpleActionClient('conversation', ConversationAction)
        self.goal = ConversationGoal()


    def execute(self, userdata):
        rospy.logwarn("Executing state Conversation")

        # Publish the mission status
        self.mission_pub.publish("Conversation")

        # Send conversation goal
        self.goal.conv_type = "give_mission"  
        rospy.logwarn(f"Started conversation \"{self.goal.conv_type}\"")
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        
        if not DEBUG:
            if result.answer:  
                # Action completed successfully
                userdata.object_id = result.answer
                return 'object_specified'
            else:
                # Action did not complete within the timeout
                return 'object_not_specified'

        else: 
            # For debugging a hardcoded object can be used
            userdata.object_id = "apple"
            return 'object_specified'


class Approach_ITEM(smach.State):
    """
    This state calls the rrt_path action server to get the path to the object.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["goal_reached", "failed_goal_reached"],
                             input_keys=["object_id"])
        
        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        # Initialize the client for the rrt_path action server
        self.rrt_path_client = actionlib.SimpleActionClient('rrt_path', MoveBaseAction)


    def execute(self, userdata):

        rospy.logwarn("Executing state Approach_ITEM")

        # Publish the mission status
        self.mission_pub.publish("Approach item")

        # Getting snapshot of the detection database
        database = rospy.wait_for_message("/spot/database", DetectionArray)
        # rospy.logwarn(f"Database was recorded as {database}")

        # Getting snapshot of the detection database
        database = rospy.wait_for_message("/spot/database", DetectionArray)
        self.goal = MoveBaseGoal()

        if not DEBUG:
            # For the real implementation the goal is set to the position of the object
            for detection in database.detections:
                if detection.type in userdata.object_id:
                    rospy.logwarn(f"Found object {detection.type} in database")
                    self.goal = detection.position

        else:      
            # For debugging a hardcoded goal can be used
            # rospy.logwarn(f"Database was recorded as {database}")
            self.goal.target_pose.pose.position.x = 45
            self.goal.target_pose.pose.position.y = 105
            self.goal.target_pose.pose.position.z = 2

        # send rrtpath goal
        self.rrt_path_client.send_goal(self.goal)
        result = self.rrt_path_client.wait_for_result(rospy.Duration(60))

        if result:  
            # Action completed successfully
            rospy.logwarn("Goal reached")
            return 'goal_reached'
        else:
            # Action did not complete within the timeout
            return 'failed_goal_reached'
        


class Approach_PERSON(smach.State):
    """
    This state calls the motion_control action server to get the path to the person.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["goal_reached", "failed_goal_reached"],
                             input_keys=["object_id"])

        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        # Initialize the client for the motion_control action server
        self.client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        self.goal = MoveBaseGoal()
        

    def execute(self, userdata):
        rospy.logwarn("Executing state Approach_PERSON")

        # Publish the mission status
        self.mission_pub.publish("Approach person")

        # Snapshot of the detection database
        database = rospy.wait_for_message("/spot/database", DetectionArray)
        # rospy.logwarn(f"Database was recorded as {database}")


        if not DEBUG:
        # For the real implementation the goal is set to the position of the object
            for detection in database.detections:
                if detection.type == "person":
                    rospy.logwarn(f"Found person in database")
                    self.goal = detection.position

        else:
        # For debugging a hardcoded goal can be used
            self.goal.target_pose.pose.position.x = 40
            self.goal.target_pose.pose.position.y = 55
            # use rrt path to object/person by coding z to 2
            self.goal.target_pose.pose.orientation.z = 2
        
        # send motioncontrol goal
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        
        if result:  
            # Action completed successfully
            rospy.logwarn("Goal reached")
            return 'goal_reached'
        else:
            # Action did not complete within the timeout
            return 'failed_goal_reached'
        
   
        return "goal_reached"

class PickItem(smach.State):
    """
    This state waits for the user to pick up the item.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["successfully_picked", "failed_more_than_3_times", "failed"])

        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        self.failed_counter = 0

    def execute(self, userdata):

        # Publish the mission status
        self.mission_pub.publish("Pick item")

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
    """
    This state waits for the user to place the item.
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=["successfully_placed", "failed_more_than_3_times", "failed"])
      
        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)
      
        self.failed_counter = 0
  
    def execute(self, userdata):

        rospy.logwarn("Please place down up the item using the controller")
        rospy.logwarn("After placing down the item, please press enter to continue")

        # Publish the mission status
        self.mission_pub.publish("Place item")

        # wait until enter key is pressed to continue, if nothing is pressed after 60 seconds then return failed
        success = wait_for_input(60)

        if success:
            return "successfully_placed"
        
        elif self.failed_counter < 3:
            return "failed"
        
        else:
            return "failed_more_than_3_times"


class ConfirmMission(smach.State):
    """
    This state calls the conversation service to confirm the mission.
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=["mission_confirmed", "mission_not_confirmed"])
                                
        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        self.client = actionlib.SimpleActionClient('conversation', ConversationAction)
        self.goal = ConversationGoal()

    def execute(self, userdata):
        rospy.logwarn("Executing state ConfirmMission")

        # Publish the mission status
        self.mission_pub.publish("Confirm mission")

        # Send conversation goal
        self.goal.conv_type = "confirm_mission"  # Change this based on your requirements
        rospy.logwarn(f"Started conversation \"{self.goal.conv_type}\"")
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration(60))
        result = self.client.get_result()
        

        if not DEBUG:
            if result.answer:  
                # Action completed successfully
                return 'mission_confirmed'
            else:
                # Action did not complete within the timeout
                return 'mission_not_confirmed'
            
        else:
            return 'mission_confirmed'


class Estop(smach.State):
    """
    This state triggers the estop.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=["triggered"])

        # Create a publisher to publish mission_status
        self.mission_pub = rospy.Publisher("/spot/mission_status", String, queue_size=10)

        # set up client to "/spot/estop/gentle" service
        self.client = rospy.ServiceProxy("/spot/estop/gentle", Trigger)

    def execute(self, userdata):

        # trigger estop
        rospy.logwarn("Estop triggered")

        # Publish the mission status
        self.mission_pub.publish("Estop triggered")

        self.client.wait_for_service()
        self.client.call(TriggerRequest())
        return "triggered"


if __name__ == '__main__':

    # Parse the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', help='Enable debug mode')
    args = parser.parse_args()
    
    # Retrieve the debug parameter value
    DEBUG = args.debug

    rospy.loginfo("Starting state machine with debug mode: " + str(DEBUG))

    # Initialize the node
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
 
