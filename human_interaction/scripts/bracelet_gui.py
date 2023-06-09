import tkinter
from ttkthemes import ThemedTk
import customtkinter 
import rospy
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from human_interaction.msg import ConversationAction, ConversationGoal, ConversationFeedback

class GUI:
    """
    A simple GUI for the SPOT control panel
    """
    def __init__(self):
        """
        Initialize the GUI
        """
        # Create a ROS action client 
        self.conv_client = actionlib.SimpleActionClient('conversation', ConversationAction)
        rospy.loginfo("Waiting for 'conversation' action server...")
        self.conv_client.wait_for_server()
        self.goal = ConversationGoal()

        self.rrt_path_client = actionlib.SimpleActionClient('rrt_path', MoveBaseAction)
        self.motion_client = actionlib.SimpleActionClient('motion_control', MoveBaseAction)
        # Create a ROS subscriber for the mission status
        sub = rospy.Subscriber("/spot/missions_status", String, self.status_callback)

        # Create a ROS service client for the soft stop
        #service_client = rospy.ServiceProxy("/spot/trigger_soft_stop", TriggerSoftStop)


        # customtkinter setup
        customtkinter.set_appearance_mode("dark")
        customtkinter.set_default_color_theme("dark-blue")

        # Create the main window
        self.window = customtkinter.CTk()
        self.window.title("SPOT Control Panel")

        # create one frame for the left half of the window
        self.left_frame = customtkinter.CTkFrame(master = self.window)
        self.left_frame.pack(side="left", fill="both", expand=True)

        # create one frame for the right half of the window
        self.right_frame = customtkinter.CTkFrame(master = self.window)
        self.right_frame.pack(side="right", fill="both", expand=True)

        # Configure the window size
        window_width = 500
        window_height = 300
        screen_width = self.window.winfo_screenwidth()
        screen_height = self.window.winfo_screenheight()
        x_coordinate = int((screen_width / 2) - (window_width / 2))
        y_coordinate = int((screen_height / 2) - (window_height / 2))
        self.window.geometry(f"{window_width}x{window_height}+{x_coordinate}+{y_coordinate}")

        # Create the left buttons
        self.button1 = customtkinter.CTkButton(self.left_frame, text="New mission (Speech)", command=self.button1_click, corner_radius=4, width=200, height=100, font=("Roboto", 25))
        self.button1.pack(pady=20)

        self.button2 = customtkinter.CTkButton(self.left_frame, text="New mission (Text)", command=self.button2_click, corner_radius=4, width=200, height=100, font=("Roboto", 25))
        self.button2.pack(pady=20)

        # Create a canvas widget
        self.canvas = tkinter.Canvas(self.right_frame, width=100, height=50)
        self.canvas.pack(pady=20)

        # Create a status label to show the current mission
        self.status_label = customtkinter.CTkLabel(self.right_frame, text="Awaiting mission status", font=("Roboto", 25))
        self.status_label.pack()

        # Create the right button for soft stop
        self.button3 = customtkinter.CTkButton(self.right_frame, text="Trigger soft stop", fg_color= "red",command=self.button3_click, corner_radius=4, width=200, height=100, font=("Roboto", 25))
        self.button3.pack(pady=40)

        # Draw a rectangle to show the status
        self.canvas.create_rectangle(0,0, 100, 50, fill="grey")

        # Remap for status color
        self.green_status = ["Idle", "Waiting for mission", "Mission completed"]
        self.yellow_status = ["Getting object", "Going to object", "Going to person", "Giving object", "Going to charging station", "Charge"]
        self.red_status = ["SPOT not available", "Mission failed"]

    
    def button1_click(self):
        """
        Callback function for the button1 click event,
        starts a new conversation with the conversation server
        """    
        self.goal.conv_type = "give_mission" 
        self.conv_client.send_goal(self.goal)
        self.conv_client.wait_for_result()
        result = self.conv_client.get_result()
        rospy.loginfo(f"Finished first conversation {self.goal.conv_type}... with result {result}")


    def button2_click(self):
        """
        Callback function for the button2 click event,
        opens a text entry field for the user to give a mission
        """
        self.button2.destroy()
        self.entry1 = customtkinter.CTkEntry(self.left_frame, placeholder_text="Which item do you want?",  width=200, height=100, font=("Roboto", 20))     
        self.entry1.pack(pady=10)
        self.entry1.bind("<Return>", self.on_enter_pressed)


    def button3_click(self):
        """
        Callback function for the button3 click event,
        triggers the soft stop
        """
        # request = TriggerSoftStopRequest()
        # response = service_client(request)
        # rospy.loginfo(f"Triggered soft stop with response {response}"
        print("Soft stop triggered")


    def on_enter_pressed(self,event):
        """
        Callback function for the enter key press in text entry field,
        replaces the text entry field with the button to give new mission,
        so the GUI either shows the button to give a new mission or the text entry field
        """
        self.entry1.destroy()
        self.button2 = customtkinter.CTkButton(self.left_frame, text="New mission (Text)", command=self.button2_click, corner_radius=2, width=200, height=100, font=("Roboto", 25))
        self.button2.pack(pady=10)
        print("pressed")


    def status_callback(self,msg):
        """
        Callback function for the mission status subscriber,
        updates the mission status label and the status rectangle accordingly
        """
        self.status_label.configure(text=f"Mission: {msg.data}")

        if msg.data in self.green_status:
            self.canvas.itemconfig(1, fill="green")

        elif msg.data in self.yellow_status:
            self.canvas.itemconfig(1, fill="yellow")
        
        elif msg.data in self.red_status:
            self.canvas.itemconfig(1, fill="red")


if __name__ == "__main__":
    """ 
    Main function, creates the GUI and runs it
    """

    # Initialize the ROS node
    rospy.init_node('bracelet_gui_node')

    # Create and run the GUI
    gui = GUI()
    gui.window.mainloop()
