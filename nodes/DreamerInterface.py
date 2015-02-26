#!/usr/bin/env python

'''
Provides an interface to Dreamer
'''

import time
import math
import rospy

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import Trajectory
import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = False

NUM_CARTESIAN_DOFS = 3 # Cartesian goal is x, y, z
NUM_ORIENTATION_DOFS = 3 # Orientation is defined using a x, y, z vector
NUM_ROBOT_DOFS = 16

class DreamerInterface:
    def __init__(self, enableUserPrompts):

        self.enableUserPrompts = enableUserPrompts

        # Define the goal messages
        rightHandCartesianGoalMsgDim = MultiArrayDimension()
        rightHandCartesianGoalMsgDim.size = NUM_CARTESIAN_DOFS
        rightHandCartesianGoalMsgDim.label = "rightHandCartesianGoal"
        rightHandCartesianGoalMsgDim.stride = 1

        self.rightHandCartesianGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_CARTESIAN_DOFS):
            self.rightHandCartesianGoalMsg.data.append(0)
        self.rightHandCartesianGoalMsg.layout.dim.append(rightHandCartesianGoalMsgDim)
        self.rightHandCartesianGoalMsg.layout.data_offset = 0
        
        #-----------------------------------------------------------------------------'

        leftHandCartesianGoalMsgDim = MultiArrayDimension()
        leftHandCartesianGoalMsgDim.size = NUM_CARTESIAN_DOFS
        leftHandCartesianGoalMsgDim.label = "leftHandCartesianGoal"
        leftHandCartesianGoalMsgDim.stride = 1

        self.leftHandCartesianGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_CARTESIAN_DOFS):
            self.leftHandCartesianGoalMsg.data.append(0)
        self.leftHandCartesianGoalMsg.layout.dim.append(leftHandCartesianGoalMsgDim)
        self.leftHandCartesianGoalMsg.layout.data_offset = 0

        #-----------------------------------------------------------------------------'

        rightHandOrientationGoalMsgDim = MultiArrayDimension()
        rightHandOrientationGoalMsgDim.size = NUM_ORIENTATION_DOFS
        rightHandOrientationGoalMsgDim.label = "rightHandOrientationGoal"
        rightHandOrientationGoalMsgDim.stride = 1

        self.rightHandOrientationGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ORIENTATION_DOFS):
            self.rightHandOrientationGoalMsg.data.append(0)
        self.rightHandOrientationGoalMsg.layout.dim.append(rightHandOrientationGoalMsgDim)
        self.rightHandOrientationGoalMsg.layout.data_offset = 0

        #-----------------------------------------------------------------------------'
        
        leftHandOrientationGoalMsgDim = MultiArrayDimension()
        leftHandOrientationGoalMsgDim.size = NUM_ORIENTATION_DOFS
        leftHandOrientationGoalMsgDim.label = "leftHandOrientationGoal"
        leftHandOrientationGoalMsgDim.stride = 1

        self.leftHandOrientationGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ORIENTATION_DOFS):
            self.leftHandOrientationGoalMsg.data.append(0)
        self.leftHandOrientationGoalMsg.layout.dim.append(leftHandOrientationGoalMsgDim)
        self.leftHandOrientationGoalMsg.layout.data_offset = 0

        #-----------------------------------------------------------------------------'
        
        postureGoalMsgDim = MultiArrayDimension()
        postureGoalMsgDim.size = NUM_ROBOT_DOFS
        postureGoalMsgDim.label = "postureGoal"
        postureGoalMsgDim.stride = 1

        self.postureGoalMsg = Float64MultiArray()
        for ii in range(0, NUM_ROBOT_DOFS):
            self.postureGoalMsg.data.append(0)
        self.postureGoalMsg.layout.dim.append(postureGoalMsgDim)
        self.postureGoalMsg.layout.data_offset = 0

        #-----------------------------------------------------------------------------'

        self.rightHandCmdMsg = Bool()
        self.rightHandCmdMsg.data = False  # relax hand

        self.rightIndexFingerCmdMsg = Bool()
        self.rightIndexFingerCmdMsg.data = True # include index finger in power grasp

        self.rightMiddleFingerCmdMsg = Bool()
        self.rightMiddleFingerCmdMsg.data = True # include middle finger in power grasp

        self.rightPinkyFingerCmdMsg = Bool()
        self.rightPinkyFingerCmdMsg.data = True # include pinky finger in power grasp            

        self.leftGripperCmdMsg = Bool()
        self.leftGripperCmdMsg.data = False  # relax gripper

        self.tareMsg = Int32()
        self.tareMsg.data = 1

        self.enableMsg = Int32()
        self.enableMsg.data = 1

        #-----------------------------------------------------------------------------'

        # Initialize member variables
        self.currentPosture = None
        self.postureError = None

        self.currentRightCartesianPos = None
        self.rightCartesianPosError = None

        self.currentLeftCartesianPos = None
        self.leftCartesianPosError = None

        self.currentRightOrientation = None
        self.rightOrientationError = None

        self.currentLeftOrientation = None
        self.leftOrientationError = None

        # Create the ROS topic subscriptions
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/Posture/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.postureTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/Posture/error",          Float64MultiArray, self.postureTaskErrorCallback)

        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.rightCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandPosition/error",               Float64MultiArray, self.rightCartesianTaskErrorCallback)

        self.leftCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandPosition/actualWorldPosition",  Float64MultiArray, self.leftCartesianTaskActualCallback)
        self.leftCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandPosition/error",                Float64MultiArray, self.leftCartesianTaskErrorCallback)

        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.rightOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandOrientation/errorAngle",    Float64,           self.rightOrientationTaskErrorCallback)

        self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualHeading", Float64MultiArray, self.leftOrientationTaskActualCallback)
        self.leftOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/errorAngle",    Float64,           self.leftOrientationTaskErrorCallback)

        # Create the ROS topic publishers
        self.postureTaskGoalPublisher = rospy.Publisher("/dreamer_controller/Posture/goalPosition", Float64MultiArray, queue_size=1)
        self.postureTaskTarePublisher = rospy.Publisher("/dreamer_controller/Posture/tare", Int32, queue_size=1)

        self.rightCartesianTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        self.rightCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/enabled", Int32, queue_size=1)
        self.rightCartesianTaskTarePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/tare", Int32, queue_size=1)

        self.leftCartesianTaskGoalPublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        self.leftCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/enabled", Int32, queue_size=1)
        self.leftCartesianTaskTarePublisher = rospy.Publisher("/dreamer_controller/LeftHandPosition/tare", Int32, queue_size=1)

        self.rightOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        self.rightOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/enabled", Int32, queue_size=1)
        self.rightOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/tare", Int32, queue_size=1)

        self.leftOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        self.leftOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/enabled", Int32, queue_size=1)
        self.leftOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/tare", Int32, queue_size=1)

        self.rightHandCmdPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/powerGrasp", Bool, queue_size=1)
        self.selectIndexFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightIndexFinger", Bool, queue_size=1)
        self.selectMiddleFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightMiddleFinger", Bool, queue_size=1)
        self.selectPinkyFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightPinkyFinger", Bool, queue_size=1)

        self.leftGripperCmdPublisher = rospy.Publisher("/dreamer_controller/controlit/leftGripper/powerGrasp", Bool, queue_size=1)
        

    def postureTaskActualCallback(self, msg):
        self.currentPosture = msg.data

    def postureTaskErrorCallback(self, msg):
        self.postureError = msg.data

    def rightCartesianTaskActualCallback(self, msg):
        self.currentRightCartesianPos = msg.data

    def rightCartesianTaskErrorCallback(self, msg):
        self.rightCartesianPosError = msg.data

    def leftCartesianTaskActualCallback(self, msg):
        self.currentLeftCartesianPos = msg.data

    def leftCartesianTaskErrorCallback(self, msg):
        self.leftCartesianPosError = msg.data

    def rightOrientationTaskActualCallback(self, msg):
        self.currentRightOrientation = msg.data

    def rightOrientationTaskErrorCallback(self, msg):
        self.rightOrientationError = msg.data

    def leftOrientationTaskActualCallback(self, msg):
        self.currentLeftOrientation = msg.data

    def leftOrientationTaskErrorCallback(self, msg):
        self.leftOrientationError = msg.data

    def getTimeSeconds(self):
        """
        Returns the current time in seconds.
        """
        return rospy.get_time()

    def connectToControlIt(self, defaultPosture):
        print "Connecting to ControlIt!..."

        # Check if already connected
        if not self.rightCartesianTaskEnablePublisher.get_num_connections() == 0:
            return not rospy.is_shutdown()

        # Wait for connection to ControlIt!
        pauseCount = 0
        warningPrinted = False

        pauseCount = 0
        warningPrinted = False
        while not rospy.is_shutdown() and (
            self.rightCartesianTaskEnablePublisher.get_num_connections() == 0 or \
            self.leftCartesianTaskEnablePublisher.get_num_connections() == 0 or \
            self.rightOrientationTaskEnablePublisher.get_num_connections() == 0 or \
            self.leftOrientationTaskEnablePublisher.get_num_connections() == 0):

            if warningPrinted:
                if self.rightCartesianTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on right cartesian enable subscriber..."
                if self.leftCartesianTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on left cartesian enable subscriber..."
                if self.rightOrientationTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on right orientation enable subscriber..."
                if self.leftOrientationTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on left orientation enable subscriber..."
                    
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not warningPrinted:
                print "Waiting for connection to ControlIt!..."
                warningPrinted = True

        if rospy.is_shutdown():
            return False

        # Enable the Cartesian position and orientation tasks
        enableMsg = Int32()
        enableMsg.data = 1        
        self.rightCartesianTaskEnablePublisher.publish(enableMsg)
        self.leftCartesianTaskEnablePublisher.publish(enableMsg)
        self.rightOrientationTaskEnablePublisher.publish(enableMsg)
        self.leftOrientationTaskEnablePublisher.publish(enableMsg)

        # Initialize the default posture
        self.postureGoalMsg.data = defaultPosture
        self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

        # Wait for connection to ControlIt!
        pauseCount = 0
        warningPrinted = False
        while not rospy.is_shutdown() and (
            self.currentPosture == None or \
            self.currentRightCartesianPos == None or self.currentLeftCartesianPos == None or \
            self.currentRightOrientation == None or self.currentLeftOrientation == None):

            if warningPrinted:
                if self.currentRightCartesianPos == None:
                    print "Waiting on right hand position state..."
                if self.currentRightOrientation == None:
                    print "Waiting on right hand orientation state..."
                if self.currentLeftCartesianPos == None:
                    print "Waiting on left hand posirition state..."
                if self.currentLeftOrientation == None:
                    print "Waiting on left hand orientation state..."
                if self.currentPosture == None:
                    print "Waiting on posture state..."

            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not warningPrinted:
                print "Waiting for data from ControlIt!..."
                warningPrinted = True

        if rospy.is_shutdown():
            return False

        print "Done connecting to ControlIt!"
        return not rospy.is_shutdown()

    def followTrajectory(self, traj):
        '''
        Follows a trajectory.

        traj - The trajectory to follow.
        duration - The amount of time to spend executing the trajectory.
        '''

        # print "Generating trajectory {0}...".format(traj.name)

        # Create the trajectory generators
        rhCartTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.rhCartWP)
        rhOrientTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.rhOrientWP)
        lhCartTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.lhCartWP)
        lhOrientTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.lhOrientWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.jPosWP)

        # Generate the trajectories
        rhCartTG.generateTrajectory(traj.duration)
        rhOrientTG.generateTrajectory(traj.duration)
        lhCartTG.generateTrajectory(traj.duration)
        lhOrientTG.generateTrajectory(traj.duration)
        jPosTG.generateTrajectory(traj.duration)

        if self.enableUserPrompts:
            index = raw_input("Execute trajectory {0}? Y/n\n".format(traj.name))
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        print "Following trajectory {0}...".format(traj.name)
        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= traj.duration:
                goalRightHandCartPos = rhCartTG.getLastPoint()
                goalRightHandOrientation = rhOrientTG.getLastPoint()
                goalLeftHandCartPos = lhCartTG.getLastPoint()
                goalLeftHandOrientation = lhOrientTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rhCartTG.getPoint(deltaTime)
                goalRightHandOrientation = rhOrientTG.getPoint(deltaTime)
                goalLeftHandCartPos = lhCartTG.getPoint(deltaTime)
                goalLeftHandOrientation = lhOrientTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        # print "Done following trajectory {0}!".format(traj.name)
        return not rospy.is_shutdown()

    def closeRightHand(self, includePinky = True, includeMiddle = True, includeIndex = True):
        
        self.rightPinkyFingerCmdMsg.data = includePinky
        self.rightMiddleFingerCmdMsg.data = includeMiddle
        self.rightIndexFingerCmdMsg.data = includeIndex
        self.rightHandCmdMsg.data = True  # close hand

        self.selectPinkyFingerPublisher.publish(self.rightPinkyFingerCmdMsg)
        self.selectMiddleFingerPublisher.publish(self.rightMiddleFingerCmdMsg)
        self.selectIndexFingerPublisher.publish(self.rightIndexFingerCmdMsg)

        self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)
        
    def openRightHand(self):
        self.rightHandCmdMsg.data = False  # open hand
        self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)
        
