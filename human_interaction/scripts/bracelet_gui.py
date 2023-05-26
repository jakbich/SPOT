from tkinter import font
from tkinter import ttk 
from ttkthemes import ThemedTk
import customtkinter 
import rospy
import actionlib

from human_interaction.msg import ConversationAction, ConversationGoal, ConversationFeedback

def button1_click():
    
    goal.conv_type = "give_mission"  # Change this based on your requirements
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Finished first conversation {goal.conv_type}... with result {result}")

def button2_click():

    goal.conv_type = "finish_mission"  # Change this based on your requirements
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo(f"Finished first conversation {goal.conv_type}... with result {result}")

def status_callback(msg):
    status_label.configure(text=f"Status: {msg.feedback}", fg="#009688")

if __name__ == "__main__":
    # Create a ROS node
    rospy.init_node('conversation_client')
    client = actionlib.SimpleActionClient('conversation', ConversationAction)
    rospy.loginfo("Waiting for 'conversation' action server...")
    client.wait_for_server()
    goal = ConversationGoal()

    sub = rospy.Subscriber("conversation/feedback", ConversationFeedback, status_callback )
   
    # customtkinter setup
    customtkinter.set_appearance_mode("dark")
    customtkinter.set_default_color_theme("dark-blue")

    # Create the main window
    window = customtkinter.CTk()
    window.title("Button GUI")

    frame = customtkinter.CTkFrame(master = window)

    # Configure the window size
    window_width = 1000
    window_height = 600
    screen_width = window.winfo_screenwidth()
    screen_height = window.winfo_screenheight()
    x_coordinate = int((screen_width / 2) - (window_width / 2))
    y_coordinate = int((screen_height / 2) - (window_height / 2))
    window.geometry(f"{window_width}x{window_height}+{x_coordinate}+{y_coordinate}")

    # Create the buttons
    button1 = customtkinter.CTkButton(window, text="Start conv 1", command=button1_click, corner_radius=2, width=150, height=30, font=("Roboto", 25))
    button1.pack(pady=10)

    button2 = customtkinter.CTkButton(window, text="Start conv 2", command=button2_click, corner_radius=2, width=150, height=30, font=("Roboto", 25))
    button2.pack(pady=10)

    

    status_label = customtkinter.CTkLabel(window, text="Status:", font=("Roboto", 25))
    status_label.pack(pady=20)
    
    entry1 = customtkinter.CTkEntry(window, placeholder_text="Which item do you want?", )
    entry1.pack(pady=20)


    # Run the GUI
    window.mainloop()
