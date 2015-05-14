#!/usr/bin/env python

'''
Takes a snapshot of Dreamer's state state.
'''

# import sys, getopt     # for getting and parsing command line arguments
import time
# import math
# import threading
import rospy

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32

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
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/Posture/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.leftCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandPosition/actualWorldPosition",  Float64MultiArray, self.leftCartesianTaskActualCallback)

        # 3-DOF orientation tasks
        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualWorldOrientation", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualWorldOrientation", Float64MultiArray, self.leftOrientationTaskActualCallback)

        # 2-DOF orientation tasks
        # self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        # self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualHeading", Float64MultiArray, self.leftOrientationTaskActualCallback)

        # Create the ROS topic publishers
        self.rightCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/enabled", Int32, queue_size=1)
        self.leftCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/enabled", Int32, queue_size=1)
        self.rightOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/enabled", Int32, queue_size=1)
        self.leftOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/enabled", Int32, queue_size=1)

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

        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (
            self.rightCartesianTaskEnablePublisher.get_num_connections() == 0 or \
            self.leftCartesianTaskEnablePublisher.get_num_connections() == 0 or \
            self.rightOrientationTaskEnablePublisher.get_num_connections() == 0 or \
            self.leftOrientationTaskEnablePublisher.get_num_connections() == 0):

            if printWarning:
                print "Waiting on connection to:"
                if self.rightCartesianTaskEnablePublisher.get_num_connections() == 0:
                    print "  - right cartesian"
                if self.leftCartesianTaskEnablePublisher.get_num_connections() == 0:
                    print "  - left cartesian"
                if self.rightOrientationTaskEnablePublisher.get_num_connections() == 0:
                    print "  - right orientation"
                if self.leftOrientationTaskEnablePublisher.get_num_connections() == 0:
                    print "  - left orientation"
                    
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for connection to ControlIt!..."
                printWarning = True

        if rospy.is_shutdown():
            return

        # Enable the Cartesian position and orientation tasks
        enableMsg = Int32()
        enableMsg.data = 1        
        self.rightCartesianTaskEnablePublisher.publish(enableMsg)
        self.leftCartesianTaskEnablePublisher.publish(enableMsg)
        self.rightOrientationTaskEnablePublisher.publish(enableMsg)
        self.leftOrientationTaskEnablePublisher.publish(enableMsg)


        # Wait for connection to ControlIt!
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (
            self.currentPosture == None or \
            self.currentRightCartesianPos == None or self.currentLeftCartesianPos == None or \
            self.currentRightOrientation == None or self.currentLeftOrientation == None):

            if printWarning:
                print "Waiting on state from:"
                if self.currentRightCartesianPos == None:
                    print "  - right hand position"
                if self.currentRightOrientation == None:
                    print "  - right hand orientation"
                if self.currentLeftCartesianPos == None:
                    print "  - left hand position"
                if self.currentLeftOrientation == None:
                    print "  - left hand orientation"
                if self.currentPosture == None:
                    print "  - posture"

            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for data from ControlIt!..."
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
            snapShot = Snapshot(self.currentRightCartesianPos, self.currentLeftCartesianPos,
                                self.currentRightOrientation, self.currentLeftOrientation, 
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

        result = result + "\n\nLeftHandOrientation:"
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
