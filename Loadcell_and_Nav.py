#!/usr/bin/env python
""" 
*** Text inside these quotation marks were not included inside the script being run ***
Purpose of this script: 
    To subscribe to the '/Limit' topic and reads the boolean flag. At the start, the 
    LIMO will traverse to the designated bookdrop site, after which the timer starts.
    
    There are 2 scenarios:
    1. If the time set (counter) has been reached, LIMO will travel back to the 
    librarian and wait for the books to be unloaded before travelling back to the site. 
    
    2. If counter has not been reached and the capacity has been reached, the counter
    will be reset (to prevent occurence of 1.). LIMO will then travel back to the 
    librarian and wait till the books has been removed before moving back to site.

    For both scenarios, the counter will reset after moving back to site.
"""
import rospy
import time
from std_msgs.msg import Bool
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

time_then = time.time() # variable for 1.5hrs wait time
once = False # variable to go to bookdrop site once at the start of the program
capacityReached = False # variable when capacity reached

class Weight_limit:
    def __init__(self):
        # Subscribes to the Ultrasonic topic 
        self.ultra_sub = rospy.Subscriber('/Limit', 
                                        Bool, self.limit_callback)

    def limit_callback(self, msg):
        global time_then, capacityReached
        # Reads true, returns to home when weight reached its limit
        if msg.data == True:
           # Resets the wait time
           time_then = time.time()
           print("Capacity reached moving to Librarian")
           self.coordinate(0.0, 0.0, 0.0)
           rospy.sleep(20) #20 seconds for the libraians to remove books
           capacityReached = True 
           
        # Check if books were removed after capacity has been reached
        if capacityReached == True and msg.data == False:
           rospy.sleep(5) #5 seconds incase librarian just removed books
           #Convenient location for students to drop off their books
           rospy.loginfo("Moving to Designated Bookdrop Site")
           LoadCell_Reader.coordinate(-1.77, -1.2, 0.0)
           # Resets the wait time
           time_then = time.time()
           capacityReached = False
           
        self.navigation()

    def navigation(self):
        global time_then, once
        if once == False:
            #Convenient location for students to drop off their books
            rospy.loginfo("Moving to Designated Bookdrop Site")
            LoadCell_Reader.coordinate(-1.77, -1.2, 0.0)
            once = True
            time_then = time.time() # Wait time begins

        # Real time... should be 1.5hrs but...
        if (time.time() - time_then) > 30:
            #Home/ bookdrop to return books or recharge
            print("Moving to Librarian")
            self.coordinates(0.0, 0.0, 0.0)
            rospy.sleep(20) #20 seconds for the libraians to remove books
            once = False # so that the LIMO goes back to bookdrop site
        
        self.limit_callback()

    def coordinate(self, x, y, z, w = 1.0):

        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        clients = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        clients.wait_for_server()
        #rospy.loginfo('Connected to SimpleActionServer: move_base')

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = w

        # Sends the goal to the action server
        clients.send_goal(goal)
        #rospy.loginfo('Executing Movement...')

        # Waits for the server to finish performing the action
        waits = clients.wait_for_result()     

        # If the result doesn't arrive, assume the Server is not available
        if not waits:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        # Initializes a rospy node to show that this script is running and visible on the ROS network
        rospy.init_node('LoadCell_And_Nav')
        rospy.loginfo("Loadcell and Nav running...")
        # Initializes the class and __init__() is called
        LoadCell_Reader = Weight_limit()

    except rospy.ROSInterruptException:
        rospy.loginfo("Program has ran it's course.")
