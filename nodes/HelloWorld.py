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

        # perform cubic spline interpolation
        tt = np.linspace(0, 30, 5)
        xx = np.array(WAYPOINT_X_SWING)
        yy = np.array(WAYPOINT_Y_SWING)
        zz = np.array(WAYPOINT_Z_SWING)

        fxx= interp1d(tt, xx, kind='cubic')    # interpolation functions
        fyy= interp1d(tt, yy, kind='cubic')
        fzz= interp1d(tt, zz, kind='cubic')

        tt_interp = np.linspace(0, 30, 1000)   # interpolate 1000 data points along trajectory
        xx_interp = fxx(tt_interp)
        yy_interp = fyy(tt_interp)
        zz_interp = fzz(tt_interp)

        # fig = plt.figure()
        # a3d = Axes3D(fig)
        # a3d.scatter(xs = xx_interp, ys = yy_interp, zs = zz_interp, zdir=u'z', label='xs=x, ys=y, zdir=z')
        # plt.show()

        # return

        R_GOALS_INTERP = []
        for ii in range(0, tt_interp.size):
            waypoint = [xx_interp[ii], yy_interp[ii], zz_interp[ii]]
            R_GOALS_INTERP.append(waypoint)

        # plt.plot(tt_interp, xx_interp, 'o', tt_interp, yy_interp, '-' , tt_interp, zz_interp, '--')
        # plt.legend(['x', 'y', 'z'], loc='best')
        # plt.show()

        # return

        goingUp = True
        goalIndex = 0

        while not rospy.is_shutdown():

            for ii in range(0, NUM_DOFS):
                self.rightHandGoalMsg.data[ii] = R_GOALS_INTERP[goalIndex][ii]
                if ii == 1:
                    self.leftHandGoalMsg.data[ii] = -1 * R_GOALS_INTERP[goalIndex][ii]
                else:
                    self.leftHandGoalMsg.data[ii] = R_GOALS_INTERP[goalIndex][ii]

            # print "publishing goal:\n  - left: {0}\n  - right: {1}".format(self.leftHandGoalMsg.data, self.rightHandGoalMsg.data)
            
            rightGoalPub.publish(self.rightHandGoalMsg)
            leftGoalPub.publish(self.leftHandGoalMsg)

            # rospy.sleep(0.03)
            rospy.sleep(0.01)

            if goalIndex == len(R_GOALS_INTERP) - 1:
                goingUp = False
            elif goalIndex == 0:
                goingUp = True

            if goingUp:
                goalIndex = goalIndex + 1
            else:
                goalIndex = goalIndex - 1


# Main method
if __name__ == "__main__":

    rospy.init_node('HelloWorldDreamer', anonymous=True)

    HelloWorldDreamer = HelloWorldDreamer()
    t = threading.Thread(target=HelloWorldDreamer.start)
    t.start()

    rospy.spin()