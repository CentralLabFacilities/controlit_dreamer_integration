#!/usr/bin/env python

'''
Publishes goals to make Dreamer flap her right hand.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# import numpy as np
# from scipy.interpolate import interp1d
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

NUM_DOFS = 3 # vector goal is x, y, z

MAX_THETA = 45.0 / 180.0 * math.pi
MIN_THETA = -45.0 / 180.0 * math.pi
INITIAL_THETA = 0.0
ANGULAR_SPEED = 100.0 / 180.0 * math.pi  # 5 degrees per second
TRAJECTORY_FREQUENCY = 100.0         # number of trajectory points to generate per second

class HandFlap:
    def __init__(self):

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = NUM_DOFS
        dim.label = "rightHandOrientationGoal"
        dim.stride = 1

        # Define the goal messages
        self.rightHandGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_DOFS):
            self.rightHandGoalMsg.data.append(0)
        self.rightHandGoalMsg.layout.dim.append(dim)
        self.rightHandGoalMsg.layout.data_offset = 0
        self.rightHandGoalMsg.data[2] = 1  # initial goal is [0, 0, 1] in world frame

        self.leftHandGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_DOFS):
            self.leftHandGoalMsg.data.append(0)
        self.leftHandGoalMsg.layout.dim.append(dim)
        self.leftHandGoalMsg.layout.data_offset = 0
        self.leftHandGoalMsg.data[2] = 1  # initial goal is [0, 0, 1] in world frame


    def getTimeSeconds(self):
        """
        Returns the current time in seconds.
        """
        return rospy.get_time()

    def start(self):
        """
        Starts publishing the goals to make Dreamer flap her right hand.
        """

        rightGoalPub = rospy.Publisher("/dreamer_controller/RightHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        leftGoalPub = rospy.Publisher("/dreamer_controller/LeftHandOrientation/goalVector", Float64MultiArray, queue_size=1)

        printGoToMax = printGoToMin = True

        # state = STATE_GO_TO_START
        # goalIndex = 0
        # numWaves = 0     # the number of times the robot has waved already

        SWEEP_TIME = (MAX_THETA - MIN_THETA) / ANGULAR_SPEED
        TOTAL_CYCLE_TIME = SWEEP_TIME * 2
        INITIAL_GO_TO_MAX_TIME = (MAX_THETA - INITIAL_THETA) / ANGULAR_SPEED

        # Offset the start time by:
        #   (1) the time to go from the min angle to the initial angle
        #   (2) the time to go from the max angle to the min angle. 
        # This is so the trajectory starts at the desired initial theta.
        startTime = self.getTimeSeconds() - (SWEEP_TIME - INITIAL_GO_TO_MAX_TIME) - SWEEP_TIME

        while not rospy.is_shutdown():

            deltaTime = self.getTimeSeconds() - startTime

            # check for completion of entire cycle
            if deltaTime > TOTAL_CYCLE_TIME:
                startTime = startTime + TOTAL_CYCLE_TIME
                deltaTime = rospy.get_time() - startTime
                printGoToMin = printGoToMax = True

            desiredAngle = 0

            if deltaTime < SWEEP_TIME:
                # go towards min theta

                if printGoToMin:
                    print "Going to min theta..."
                    printGoToMin = False

                deltaAngle = ANGULAR_SPEED * deltaTime
                desiredAngle = MAX_THETA - deltaAngle

            else:
                # go towards max theta
                if printGoToMax:
                    print "Going to max theta..."
                    printGoToMax = False

                deltaTime = deltaTime - SWEEP_TIME
                deltaAngle = ANGULAR_SPEED * deltaTime
                desiredAngle = MIN_THETA + deltaAngle

            self.leftHandGoalMsg.data[0] = self.rightHandGoalMsg.data[0] = math.sin(desiredAngle)
            self.leftHandGoalMsg.data[2] = self.rightHandGoalMsg.data[2] = math.cos(desiredAngle)

            rightGoalPub.publish(self.rightHandGoalMsg)
            leftGoalPub.publish(self.leftHandGoalMsg)

            time.sleep(1.0 / TRAJECTORY_FREQUENCY)

# Main method
if __name__ == "__main__":

    rospy.init_node('HandFlap', anonymous=True)

    HandFlap = HandFlap()
    t = threading.Thread(target=HandFlap.start)
    t.start()

    rospy.spin()
