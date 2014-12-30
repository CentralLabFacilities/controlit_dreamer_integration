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

            rospy.sleep(2)

            if goalIndex == len(R_GOALS) - 1:
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