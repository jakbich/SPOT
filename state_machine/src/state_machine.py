#!/usr/bin/env python

import rospy
import smach
import smach_ros

initialization = True
searching = True
trajectoryexecution = True
help_needed = True
pick_item = True
item_conformation = True
action_confirmed = True

# Initialization state
class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["initialization_succeeded", "initialization_aborted"])

    def execute(self, userdata):
        rospy.loginfo("Executing state Initialization")
        if initialization:
            return "initialization_succeeded"
        else:
            return "initialization_aborted"   

# define state Searching
class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["search_succeeded", "search_aborted"],
                             input_keys=["item_id"],
                             output_keys=["item_location"])

    def execute(self, userdata):
        rospy.loginfo("Executing state Search, searching for: %s", userdata.item_id)
        if searching:
            if userdata.item_id == "Person":
                userdata.item_location = [5, 5]
            elif userdata.item_id == "Apple":
                userdata.item_location = [1, 1]

            return "search_succeeded"
        else:
            return "search_aborted"
        
# define state TrajectoryExecution
class TrajectoryExecution(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["item_reached", "returned_to_person", "person_reached", "moved_aborted"],
                             input_keys=["item_id", "goal_location", "item_holding"])

    def execute(self, userdata):
        rospy.loginfo("Executing state TrajectoryExecution, goal: %s", str(userdata.goal_location))
        if trajectoryexecution:
            if userdata.item_id == "Apple":
                return "item_reached"
            elif userdata.item_holding == True:
                return "returned_to_person"
            else:
                return "person_reached"
        else:
            return "moved_aborted"
        
# define state HelpNeeded
class HelpNeeded(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["help_needed", "no_help_needed"],
                             output_keys=["item_id"])

    def execute(self, userdata):
        rospy.loginfo("Executing state HelpNeeded")
        if help_needed:
            userdata.item_id = "Apple"
            return "help_needed"
        else:
            return "no_help_needed"
        
# define state PickItem
class PickItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["pick_succeeded", "pick_aborted"],
                             input_keys=["goal_location"],
                             output_keys=["item_id", "item_holding"])

    def execute(self, userdata):
        rospy.loginfo("Executing state PickItem, item location: %s", str(userdata.goal_location))
        if pick_item:
            userdata.item_holding = True
            userdata.item_id = "Person"
            return "pick_succeeded"
        else:
            return "pick_aborted"
        
# define state ItemConformation
class ItemConformation(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["item_confirmed", "item_not_confirmed"],
                             input_keys=["item_id"])

    def execute(self, userdata):
        rospy.loginfo("Executing state ItemConformation, item id: %s", str(userdata.item_id))
        if item_conformation:
            return "item_confirmed"
        else:
            return "item_not_confirmed"
        
# define state ItemConformation
class GiveItem(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["give_item_succeeded", "give_item_aborted"],
                             input_keys=["goal_location"],
                             output_keys=["item_holding"])

    def execute(self, userdata):
        rospy.loginfo("Executing state GiveItem, location: %s", str(userdata.goal_location))
        if item_conformation:
            userdata.item_holding = False
            return "give_item_succeeded"
        else:
            return "give_item_aborted"
        
# define state ConfirmAction
class ConfirmAction(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=["action_confirmed", "action_not_confirmed"])

    def execute(self, userdata):
        rospy.loginfo("Executing state ConfirmAction")
        if action_confirmed:
            return "action_confirmed"
        else:
            return "action_not_confirmed"



if __name__ == '__main__':
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["finished"])

    sm.userdata.item_id = "Person"
    sm.userdata.item_location = [0, 0]
    sm.userdata.item_holding = False

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add("INITIALIZATION", Initialization(), 
                               transitions={"initialization_succeeded": "SEREACH", 
                                            "initialization_aborted": "finished"})
        
        smach.StateMachine.add("SEREACH", Search(), 
                               transitions={"search_succeeded": "APPROCH_GOAL", 
                                            "search_aborted": "finished"},
                               remapping={"item_id": "item_id",
                                          "item_location": "item_location"})
        
        smach.StateMachine.add("APPROCH_GOAL", TrajectoryExecution(), 
                               transitions={"item_reached": "PICK_ITEM",
                                            "returned_to_person": "ITEM_CONFORMATION",
                                            "person_reached": "HELP_NEEDED",
                                            "moved_aborted": "finished"},
                               remapping={"item_id": "item_id",
                                          "goal_location": "item_location",
                                          "item_holding": "item_holding"})
        
        smach.StateMachine.add("HELP_NEEDED", HelpNeeded(), 
                               transitions={"help_needed": "SEREACH", 
                                            "no_help_needed": "finished"},
                               remapping={"item_id": "item_id"})
        
        smach.StateMachine.add("PICK_ITEM", PickItem(), 
                               transitions={"pick_succeeded": "APPROCH_GOAL", 
                                            "pick_aborted": "finished"},
                               remapping={"item_id": "item_id",
                                          "goal_location": "item_location",
                                          "item_holding": "item_holding"})
        
        smach.StateMachine.add("ITEM_CONFORMATION", ItemConformation(), 
                               transitions={"item_confirmed": "GIVE_ITEM", 
                                            "item_not_confirmed": "finished"},
                               remapping={"item_id": "item_id"})
        
        smach.StateMachine.add("GIVE_ITEM", GiveItem(), 
                               transitions={"give_item_succeeded": "CONFIRM_ACTION", 
                                            "give_item_aborted": "finished"},
                               remapping={"goal_location": "item_location",
                                          "item_holding": "item_holding"})
        
        smach.StateMachine.add("CONFIRM_ACTION", ConfirmAction(), 
                               transitions={"action_confirmed": "finished", 
                                            "action_not_confirmed": "finished"})
        
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
