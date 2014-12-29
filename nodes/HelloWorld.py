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

NUM_DOFS = 3 # Cartesian goal is x, y, z

# These are the right hand Cartesian goal positions.
# Left hand goal positions simply negate the Y axis goal.
R_GOALS = [[0.03607483951774613, -0.6034890864442964, 1.0984258963904037],
           [0.05515703618521882, -0.6566814061330449, 1.4456665344232338],
           [0.018828338330071966, -0.45512000705525885, 1.74547219397573]]

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

        goingUp = True
        goalIndex = 0

        while not rospy.is_shutdown():

            for ii in range(0, NUM_DOFS):
                self.rightHandGoalMsg.data[ii] = R_GOALS[goalIndex][ii]
                if ii == 1:
                    self.leftHandGoalMsg.data[ii] = -1 * R_GOALS[goalIndex][ii]
                else:
                    self.leftHandGoalMsg.data[ii] = R_GOALS[goalIndex][ii]

            rightGoalPub.publish(self.rightHandGoalMsg)
            leftGoalPub.publish(self.leftHandGoalMsg)

            rospy.sleep(10)

            if goalIndex == 2:
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