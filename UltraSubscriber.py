#!/usr/bin/env python
""" 
*** Text inside these quotation marks were not included inside the script being run ***
Purpose of this script: 
    To subscribe to the '/Ultra' topic and reads the boolean flag, when the flag shows 
    true, all movement is stopped by publishing to '/cmd_vel' topic which has a 
    higher priority compared to move_base. Stoppage continues till the obstacle is no
    longer detected.    
"""
import rospy
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Ultrasonic_detector:
    def __init__(self):
        # Subscribes to the Ultrasonic topic 
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                        Twist, queue_size=6)
        self.twist = Twist()
        self.ultra_sub = rospy.Subscriber('/Ultra', 
                                        Bool, self.ultra_callback)

    def ultra_callback(self, msg):
        # Reads msg that reflects if sensors detect obstacles
        self.ultra_detected = msg.data
        # Reads true, obstacles detected, stops all movements
        while self.ultra_detected == True:
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
        
        # Recursion
        self.ultra_callback()
	    # when no obstacles are detected, no stopping is required

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
    	
        # Initializes a rospy node to show that this script is running and visible on the ROS network
        rospy.init_node('Ultrasonic_Reader')
        rospy.loginfo("Ultrasonic subscriber running...")
        # Initializes the class and __init__() is called
        Ultrasonic_Reader = Ultrasonic_detector()

    except rospy.ROSInterruptException:
        rospy.loginfo("Program has ran it's course.")
