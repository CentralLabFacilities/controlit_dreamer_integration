#!/usr/bin/env python

'''
Publishes goals to make Dreamer wave her arms.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

NUM_DOFS = 3 # Cartesian goal is x, y, z

# These are the right hand Cartesian goal positions.
# Left hand goal positions simply negate the Y axis goal.
R_GOALS = [[0.45, -0.25, 1.1],        # move right hand to the right
           [0.438, -0.356, 1.106],
           [0.375, -0.451, 1.102],
           [0.291, -0.508, 1.092],
           [0.144, -0.528, 1.049],

           [0.13829324246419664, -0.5779865808605724, 1.1269252743334133], # move right hand up
           [0.14043433470588146, -0.601732376320113, 1.1971891610582215],
           [0.1434783706196868, -0.6189997526862935, 1.2998303686016914],
           [0.14583955739422813, -0.6231028442528961, 1.3856296718569716],
           [0.1481883320341411, -0.6013934906799016, 1.4712865537773556],
           [0.1500215472226119, -0.568639980698996, 1.5753674402311533]]

# These values were taken from R_GOALS above
WAYPOINT_X_SWING = [0.45, 0.438, 0.375, 0.291, 0.144]
WAYPOINT_Y_SWING = [-0.25, -0.356, -0.451, -0.508, -0.528]
WAYPOINT_Z_SWING = [1.1, 1.106, 1.102, 1.092, 1.049]

WAYPOINT_X_WAVE = [0.138, 0.140, 0.143, 0.146, 0.148, 0.150]
WAYPOINT_Y_WAVE = [-0.578, -0.601, -0.619, -0.623, -0.601, -0.569]
WAYPOINT_Z_WAVE = [1.127, 1.197, 1.300, 1.386, 1.471, 1.575]

# Define FSM States and constants
STATE_GO_TO_START = 0
STATE_WAVE_UP = 1
STATE_WAVE_DOWN = 2
STATE_GO_TO_FINISH = 3

NUM_WAVES = 3
Y_AXIS_INDEX = 1

class HelloWorldDreamer:
    def __init__(self):

        # Define the dimensions of the message
        dim = MultiArrayDimension()
        dim.size = NUM_DOFS
        dim.label = "rightHandGoal"
        dim.stride = 1

        # Define the goal messages
        self.rightHandGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_DOFS):
            self.rightHandGoalMsg.data.append(0)
        self.rightHandGoalMsg.layout.dim.append(dim)
        self.rightHandGoalMsg.layout.data_offset = 0

        dim.label = "leftHandGoal"
        self.leftHandGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_DOFS):
            self.leftHandGoalMsg.data.append(0)
        self.leftHandGoalMsg.layout.dim.append(dim)
        self.leftHandGoalMsg.layout.data_offset = 0

    def start(self):
        """
        Starts publishing the goals to make Dreamer wave her arms.
        """

        rightGoalPub = rospy.Publisher("/dreamer_controller/RightHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        leftGoalPub = rospy.Publisher("/dreamer_controller/LeftHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        startTime = time.time()

        # Perform cubic spline interpolation for initial swing
        tt = np.linspace(0, 30, 5)             # start, end, number of values
        xx = np.array(WAYPOINT_X_SWING)
        yy = np.array(WAYPOINT_Y_SWING)
        zz = np.array(WAYPOINT_Z_SWING)

        fxx = interp1d(tt, xx, kind='cubic')   # interpolation functions
        fyy = interp1d(tt, yy, kind='cubic')
        fzz = interp1d(tt, zz, kind='cubic')

        tt_interp = np.linspace(0, 30, 1000)   # interpolate 1000 data points along trajectory
        xx_interp = fxx(tt_interp)
        yy_interp = fyy(tt_interp)
        zz_interp = fzz(tt_interp)

        # fig = plt.figure()
        # a3d = Axes3D(fig)
        # a3d.scatter(xs = xx_interp, ys = yy_interp, zs = zz_interp, zdir=u'z', label='xs=x, ys=y, zdir=z')
        # plt.show()

        # return

        # Save the interpolated coordinates into a new datastructure
        INITIAL_SWING_TRAJECTORY = []
        for ii in range(0, tt_interp.size):
            waypoint = [xx_interp[ii], yy_interp[ii], zz_interp[ii]]
            INITIAL_SWING_TRAJECTORY.append(waypoint)

        # plt.plot(tt_interp, xx_interp, 'o', tt_interp, yy_interp, '-' , tt_interp, zz_interp, '--')
        # plt.legend(['x', 'y', 'z'], loc='best')
        # plt.show()

        # return

        # Perform cubic spline interpolation for wave
        tt = np.linspace(0, 60, 6)             # start, end, number of values
        xx = np.array(WAYPOINT_X_WAVE)
        yy = np.array(WAYPOINT_Y_WAVE)
        zz = np.array(WAYPOINT_Z_WAVE)

        fxx= interp1d(tt, xx, kind='cubic')    # interpolation functions
        fyy= interp1d(tt, yy, kind='cubic')
        fzz= interp1d(tt, zz, kind='cubic')

        tt_interp = np.linspace(0, 60, 2000)   # interpolate 2000 data points along trajectory
        xx_interp = fxx(tt_interp)
        yy_interp = fyy(tt_interp)
        zz_interp = fzz(tt_interp)

        # Save the interpolated coordinates into a new datastructure
        WAVE_TRAJECTORY = []
        for ii in range(0, tt_interp.size):
            waypoint = [xx_interp[ii], yy_interp[ii], zz_interp[ii]]
            WAVE_TRAJECTORY.append(waypoint)


        print "Starting on STATE_GO_TO_START"
        state = STATE_GO_TO_START
        goalIndex = 0
        numWaves = 0     # the number of times the robot has waved already

        while not rospy.is_shutdown():

            if state == STATE_GO_TO_START:
                for ii in range(0, NUM_DOFS):
                    self.rightHandGoalMsg.data[ii] = INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                    if ii == Y_AXIS_INDEX:
                        self.leftHandGoalMsg.data[ii] = -1 * INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                    else:
                        self.leftHandGoalMsg.data[ii] = INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                goalIndex = goalIndex + 1
                if goalIndex == len(INITIAL_SWING_TRAJECTORY):
                    print "Transitioning to STATE_WAVE_UP"
                    state = STATE_WAVE_UP
                    goalIndex = 0
            
            elif state == STATE_WAVE_UP:
                for ii in range(0, NUM_DOFS):
                    self.rightHandGoalMsg.data[ii] = WAVE_TRAJECTORY[goalIndex][ii]
                    if ii == Y_AXIS_INDEX:
                        self.leftHandGoalMsg.data[ii] = -1 * WAVE_TRAJECTORY[goalIndex][ii]
                    else:
                        self.leftHandGoalMsg.data[ii] = WAVE_TRAJECTORY[goalIndex][ii]
                goalIndex = goalIndex + 1
                if goalIndex == len(WAVE_TRAJECTORY):
                    print "Transitioning to STATE_WAVE_DOWN"
                    state = STATE_WAVE_DOWN
                    goalIndex = len(WAVE_TRAJECTORY) - 1
            
            elif state == STATE_WAVE_DOWN:
                for ii in range(0, NUM_DOFS):
                    self.rightHandGoalMsg.data[ii] = WAVE_TRAJECTORY[goalIndex][ii]
                    if ii == Y_AXIS_INDEX:
                        self.leftHandGoalMsg.data[ii] = -1 * WAVE_TRAJECTORY[goalIndex][ii]
                    else:
                        self.leftHandGoalMsg.data[ii] = WAVE_TRAJECTORY[goalIndex][ii]
                goalIndex = goalIndex - 1
                if goalIndex == -1:
                    numWaves = numWaves + 1
                    if numWaves == NUM_WAVES:
                        print "Transitioning to STATE_WAVE_FINISH"
                        state = STATE_GO_TO_FINISH
                        numWaves = 0
                        goalIndex = len(INITIAL_SWING_TRAJECTORY) - 1
                    else:
                        print "Transitioning to STATE_WAVE_UP, numWaves = {0}".format(numWaves)
                        state = STATE_WAVE_UP
                        goalIndex = 0

            else:
                for ii in range(0, NUM_DOFS):
                    self.rightHandGoalMsg.data[ii] = INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                    if ii == Y_AXIS_INDEX:
                        self.leftHandGoalMsg.data[ii] = -1 * INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                    else:
                        self.leftHandGoalMsg.data[ii] = INITIAL_SWING_TRAJECTORY[goalIndex][ii]
                goalIndex = goalIndex - 1
                if goalIndex == -1:
                    print "Transitioning to STATE_GO_TO_START"
                    state = STATE_GO_TO_START
                    goalIndex = 0            

            # print "publishing goal:\n  - left: {0}\n  - right: {1}".format(self.leftHandGoalMsg.data, self.rightHandGoalMsg.data)

            rightGoalPub.publish(self.rightHandGoalMsg)
            leftGoalPub.publish(self.leftHandGoalMsg)

            if state == STATE_GO_TO_START or state == STATE_GO_TO_FINISH:
                # rospy.sleep(0.03)
                rospy.sleep(0.01)      # 10 seconds for this initial / final trajectory
            else:
                rospy.sleep(0.01)      # 20 seconds for this wave trajectory

# Main method
if __name__ == "__main__":

    rospy.init_node('HelloWorldDreamer', anonymous=True)

    HelloWorldDreamer = HelloWorldDreamer()
    t = threading.Thread(target=HelloWorldDreamer.start)
    t.start()

    rospy.spin()