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

class Snapshot:
    def __init__(self, rightHandCartesian, leftHandCartesian, rightHandOrientation, leftHandOrientation, posture):
        self.rightHandCartesian = rightHandCartesian
        self.leftHandCartesian = leftHandCartesian
        self.rightHandOrientation = rightHandOrientation
        self.leftHandOrientation = leftHandOrientation
        self.posture = posture

    def __str__(self):
        return "Snapshot:\n"\
                  "  - right hand Cartesian position: {0}\n"\
                  "  - right hand orientation: {1}\n"\
                  "  - left hand Cartesian position: {2}\n"\
                  "  - left hand orientation: {3}\n"\
                  "  - posture: {4}".format(self.rightHandCartesian, self.rightHandOrientation,
                                            self.leftHandCartesian, self.leftHandOrientation, 
                                            self.posture)

    def __repr__(self):
        return self.__str__()

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

        # Wait for robot to be placed into desired position
        counter = 5
        while not rospy.is_shutdown() and counter > 0:
            print "Taking snapshot in {0}...".format(counter)
            time.sleep(1)
            counter = counter - 1

        if rospy.is_shutdown():
            return

        snapShots = []

        done = False
        while not done:

            if rospy.is_shutdown():
                return
    
            # Take the snapshot
            snapShot = Snapshot(self.currentRightCartesianPos, self.currentRightOrientation,
                                self.currentLeftCartesianPos, self.currentLeftOrientation, 
                                self.currentPosture)
            print snapShot
            snapShots.append(snapShot)

            index = raw_input("Take another snapshot? Y/n\n")

            if index == "N" or index == "n":
                done = True

            if rospy.is_shutdown():
                return

        # Print the results
        result = " === Forward trajectories ==="
        
        result = result + "\n\nRightHandPosition:"
        for snapshot in snapShots:
            result = result + "\n  " 
            result = result + "{0}".format(snapshot.rightHandCartesian)

        result = result + "\nRightHandOrientation:"
        for snapshot in snapShots:
            result = result + "\n  "
            result = result + "{0}".format(snapshot.rightHandOrientation)

        result = result + "\nLeftHandPosition:"
        for snapshot in snapShots:
            result = result + "\n  "
            result = result + "{0}".format(snapshot.leftHandCartesian)

        result = result + "\nLeftHandOrientation:"
        for snapshot in snapShots:
            result = result + "\n  "
            result = result + "{0}".format(snapshot.leftHandOrientation)

        result = result + "\nPosture:"
        for snapshot in snapShots:
            result = result + "\n  "
            result = result + "{0}".format(snapshot.posture)

        result = result + "\n\n === Reverse trajectories ==="
        
        result = result + "\n\nRightHandPosition:"
        for snapshot in snapShots[::-1]:
            result = result + "\n  " 
            result = result + "{0}".format(snapshot.rightHandCartesian)

        result = result + "\n\nRightHandOrientation:"
        for snapshot in snapShots[::-1]:
            result = result + "\n  " 
            result = result + "{0}".format(snapshot.rightHandOrientation)

        result = result + "\n\nLeftHandPosition:"
        for snapshot in snapShots[::-1]:
            result = result + "\n  " 
            result = result + "{0}".format(snapshot.leftHandCartesian)

        result = result + "\n\nnLeftHandOrientation:"
        for snapshot in snapShots[::-1]:
            result = result + "\n  " 
            result = result + "{0}".format(snapshot.leftHandOrientation)

        result = result + "\nPosture:"
        for snapshot in snapShots[::-1]:
            result = result + "\n  "
            result = result + "{0}".format(snapshot.posture)

        print result


# Main method
if __name__ == "__main__":

    rospy.init_node('TakeSnapshot', anonymous=True)

    snapshot = TakeSnapshot()
    snapshot.run()