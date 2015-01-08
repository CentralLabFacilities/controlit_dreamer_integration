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
    def __init__(self, rosTopic, includeFingerSelect = False):
        """
        The constructor.

        Keyword arguments:
        rosTopic -- The ROS topic on which to publish the power grasp command.
        """

        self.rosTopic = rosTopic

        print "Creating ROS publisher on topic {0}".format(self.rosTopic)
        self.publisher = rospy.Publisher(self.rosTopic, Bool, queue_size=1)
        
        if includeFingerSelect:
            self.selectIndexFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightIndexFinger", Bool, queue_size=1)
            self.selectMiddleFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightMiddleFinger", Bool, queue_size=1)
            self.selectPinkyFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightPinkyFinger", Bool, queue_size=1)

        # Define the power grasp command message
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

    def enableDisableIndexFinger(self):
        """
        Enables or disables the right index finger.
        """
        index = raw_input("Include right index finger in power grasp? Y/n\n")
        if not (index == "N" or index == "n"):
            self.commandMsg.data = True  # include the finger
        else:
            self.commandMsg.data = False
        self.selectIndexFingerPublisher.publish(self.commandMsg)

    def enableDisableMiddleFinger(self):
        """
        Enables or disables the right middle finger.
        """
        index = raw_input("Include right middle finger in power grasp? Y/n\n")
        if not (index == "N" or index == "n"):
            self.commandMsg.data = True  # include the finger
        else:
            self.commandMsg.data = False
        self.selectMiddleFingerPublisher.publish(self.commandMsg)

    def enableDisablePinkyFinger(self):
        """
        Enables or disables the right pinky finger.
        """
        index = raw_input("Include right pinky finger in power grasp? Y/n\n")
        if not (index == "N" or index == "n"):
            self.commandMsg.data = True  # include the finger
        else:
            self.commandMsg.data = False
        self.selectPinkyFingerPublisher.publish(self.commandMsg)

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
    rightHandPowerGraspControl = RightHandPowerGraspControl(rosTopic = rosTopic, includeFingerSelect = True)
    t = threading.Thread(target=rightHandPowerGraspControl.start)
    t.start()

    done = False
    while not done:
        index = raw_input("Right hand power grasp state (type t, f, i, m, p, or q to exit): ")

        if "q" == index:
            done = True
        elif "t" == index:
            rightHandPowerGraspControl.doPowerGrasp()
        elif "f" == index:
            rightHandPowerGraspControl.relaxPowerGrasp()
        elif "i" == index:
            # enable / disable index finger
            rightHandPowerGraspControl.enableDisableIndexFinger();
        elif "m" == index:
            # enable / disable middle finger
            rightHandPowerGraspControl.enableDisableMiddleFinger();
        elif "p" == index:
            # enable / disable pinky finger
            rightHandPowerGraspControl.enableDisablePinkyFinger();
        else:
            print "Unknown command {0}".format(index)

    rospy.signal_shutdown("done")
