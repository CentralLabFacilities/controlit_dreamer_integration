#!/usr/bin/env python

'''
This demo is for the UT Austin EERC Groundbreaking ceremony to be
held on Feb 26, 2015 in front of RLM.

To start the hand wave demo:
  $ rostopic pub --once /demo8/cmd std_msgs/Int32 'data: 0'
'''

import sys, getopt     # for getting and parsing command line arguments
# import time
# import math
# import threading
import rospy

from std_msgs.msg import Int32

import Demo4_HandWave
import Demo5_HandShake
import Demo7_HookemHorns

# import numpy as np
# from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

import DreamerInterface
# import Trajectory
# import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = False

DEMO_NONE = -1
DEMO_WAVE = 0
DEMO_SHAKE = 1
DEMO_HOOKEM_HORNS = 2

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

class Demo8_EERC_Groundbreaking_Ceremony:
    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(ENABLE_USER_PROMPTS)
        self.handWaveDemo = Demo4_HandWave.Demo4_HandWave(self.dreamerInterface)
        self.handShakeDemo = Demo5_HandShake.Demo5_HandShake(self.dreamerInterface)
        self.hookemHornsDemo = Demo7_HookemHorns.Demo7_HookemHorns(self.dreamerInterface)

        self.runDemo = False
        self.demoNumber = DEMO_NONE

        self.demoCmdSubscriber  = rospy.Subscriber("/demo8/cmd", Int32, self.demoCmdCallback)
        self.demoDonePublisher = rospy.Publisher("/demo8/done",  Int32, queue_size=1)

        self.doneMessage = Int32()
        self.doneMessage.data = 1

    def demoCmdCallback(self, msg):
        self.demoNumber = msg.data
        self.runDemo = True

    def run(self):
        while not rospy.is_shutdown():
            if self.runDemo:
                if self.demoNumber == DEMO_WAVE:
                    print "Starting the Hand Wave Demo!"
                    self.handWaveDemo.run(enablePrompts = False)
                elif self.demoNumber == DEMO_SHAKE:
                    print "Starting the Hand Shake Demo!"
                    self.handShakeDemo.run(enablePrompts = False)
                elif self.demoNumber == DEMO_HOOKEM_HORNS:
                    print "Starting the Hook'em Horns Demo!"
                    self.hookemHornsDemo.run(enablePrompts = False)

                print "Done executing demo. Publishing done message."
                self.demoDonePublisher.publish(self.doneMessage)

                self.runDemo = False
                self.demoNumber = DEMO_NONE

            rospy.sleep(0.1)

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo8_EERC_Groundbreaking_Ceremony', anonymous=True)

    demo = Demo8_EERC_Groundbreaking_Ceremony()
    demo.run()

    print "Demo 8 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting
