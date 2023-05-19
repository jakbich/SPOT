import rospy
import actionlib

from human_interaction.msg import ConversationAction, ConversationGoal

if __name__ == '__main__':

    rospy.init_node('conversation_client')
    client = actionlib.SimpleActionClient('conversation', ConversationAction)
    rospy.loginfo("Waiting for 'conversation' action server...")
    client.wait_for_server()
    goal = ConversationGoal()

    # Set the desired conversation type ('new' or 'old')
    goal.conv_type = "give_mission"  # Change this based on your requirements
    rospy.loginfo(f"Started conversation \"{goal.conv_type}\"")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Finished first conversation {goal.conv_type}... with result {result}")


    
    # Set the desired conversation type ('new' or 'old')
    goal.conv_type = 'finish_mission'  # Change this based on your requirements
    rospy.loginfo(f"Started conversation \"{goal.conv_type}\"")
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Finished first conversation {goal.conv_type}... with result {result}")
