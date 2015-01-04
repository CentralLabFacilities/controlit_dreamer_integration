#!/usr/bin/env python

'''
Allows a user to control Dreamer's right hand power grasp.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
from std_msgs.msg import Bool

class RightHandPowerGraspControl:
    def __init__(self, rosTopic):
        """
        The constructor.

        Keyword arguments:
        rosTopic -- The ROS topic on which to publish the power grasp command.
        """

        self.rosTopic = rosTopic

        print "Creating ROS publisher on topic {0}".format(self.rosTopic)
        self.publisher = rospy.Publisher(self.rosTopic, Bool, queue_size=1)
        
        # Define the power grasm command message
        self.commandMsg = Bool()
        self.commandMsg.data = False

    def doPowerGrasp(self):
        """
        Enables the power grasp.
        """
        self.commandMsg.data = True
        self.publisher.publish(self.commandMsg)
        

    def relaxPowerGrasp(self):
        """
        Disables the power grasp.
        """
        self.commandMsg.data = False
        self.publisher.publish(self.commandMsg)

    def start(self):
        """
        Starts the power grasp control thread.
        """
        rospy.spin() # prevent thread from exiting


# Main method
if __name__ == "__main__":

    rospy.init_node('RightHandPowerGraspControl', anonymous=True)

    # Define default values for the command line arguments
    rosTopic = '/dreamer_controller/controlit/rightHand/powerGrasp'
    
    usageStr = "Usage: python RightHandPowerGraspControl.py [parameters]\n"\
               "Valid parameters include:\n"\
               " -h\n"\
               " -t or --rosTopic [ros topic] (default {0})".format(rosTopic)

    # Parse the command line arguments
    try:
        opts, args = getopt.getopt(sys.argv[1:],"ht:",["rosTopic="])
    except getopt.GetoptError:
       rospy.logerr(usageStr)
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            rospy.loginfo(usageStr)
            sys.exit()
        elif opt in ("-t", "--rosTopic"):
            rosTopic = arg
        else:
            print "Unknown argument \"{0}\"".format(opt)

    rospy.loginfo("Input parameters:\n"\
                  "  - ROS topic: {0}".format(rosTopic))

    # Create a RightHandPowerGraspControl object
    rightHandPowerGraspControl = RightHandPowerGraspControl(rosTopic)
    t = threading.Thread(target=rightHandPowerGraspControl.start)
    t.start()

    done = False
    while not done:
        index = raw_input("Right hand power grasp state (type t, f, or q to exit): ")

        if "q" == index:
            done = True
        elif "t" == index:
            rightHandPowerGraspControl.doPowerGrasp()
        elif "f" == index:
            rightHandPowerGraspControl.relaxPowerGrasp()
        else:
            print "Unknown command {0}".format(index)

    rospy.signal_shutdown("done")
