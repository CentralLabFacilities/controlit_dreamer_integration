#!/usr/bin/env python

'''
Takes a snapshot of Dreamer's state state.
'''

# import sys, getopt     # for getting and parsing command line arguments
import time
# import math
# import threading
import rospy

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class TakeSnapshot:
    def __init__(self):

        # Initialize member variables
        self.currentPosture = None
        self.currentRightCartesianPos = None
        self.currentLeftCartesianPos = None
        self.currentRightOrientation = None
        self.currentLeftOrientation = None

        # Create the ROS topic subscriptions
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/JPosTask/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.leftCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandPosition/actualWorldPosition",  Float64MultiArray, self.leftCartesianTaskActualCallback)
        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualHeading", Float64MultiArray, self.leftOrientationTaskActualCallback)

    def postureTaskActualCallback(self, msg):
        self.currentPosture = msg.data

    def rightCartesianTaskActualCallback(self, msg):
        self.currentRightCartesianPos = msg.data

    def leftCartesianTaskActualCallback(self, msg):
        self.currentLeftCartesianPos = msg.data

    def rightOrientationTaskActualCallback(self, msg):
        self.currentRightOrientation = msg.data

    def leftOrientationTaskActualCallback(self, msg):
        self.currentLeftOrientation = msg.data

    def run(self):
        """
        Runs the snapshot
        """

        # Wait for connection to ControlIt!
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (
            self.currentPosture == None or \
            self.currentRightCartesianPos == None or self.currentLeftCartesianPos == None or \
            self.currentRightOrientation == None or self.currentLeftOrientation == None):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for connection to ControlIt!..."
                printWarning = True

        if rospy.is_shutdown():
            return

        done = False

        while not done:

            # Wait 10 seconds for robot to be placed into desired position
            counter = 10
            while counter > 0:
                print "Taking snapshot in {0}...".format(counter)
                time.sleep(1)
                counter = counter - 1
    
            # Take the snapshot
            print "Snapshot:\n"\
                  "  - right hand Cartesian position: {0}\n"\
                  "  - right hand orientation: {1}\n"\
                  "  - left hand Cartesian position: {2}\n"\
                  "  - left hand orientation: {3}\n"\
                  "  - posture: {4}".format(self.currentRightCartesianPos, self.currentRightOrientation,
                                            self.currentLeftCartesianPos, self.currentLeftOrientation, 
                                            self.currentPosture)

            index = raw_input("Take another snapshot? y/N")

            if index == "y" or index == "Y":
                done = False
            else:
                done = True

# Main method
if __name__ == "__main__":

    rospy.init_node('TakeSnapshot', anonymous=True)

    snapshot = TakeSnapshot()
    snapshot.run()