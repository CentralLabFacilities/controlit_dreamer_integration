#!/usr/bin/env python

'''
Provides an interface to Dreamer
'''

import time
import math
import rospy

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32
from dreamer_controlit_common.msg import QuatInterpMsg

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import Trajectory
import TrajectoryGeneratorCubicSpline


ENABLE_USER_PROMPTS = False

NUM_CARTESIAN_DOFS = 3 # Cartesian goal is x, y, z
# NUM_ORIENTATION_DOFS = 3 
NUM_ROBOT_DOFS = 16

class DreamerInterface:
    def __init__(self, enableUserPrompts = False, useQuaternionControl = False):
        '''
        The constructor.

        Keyword arguments:
          - enableUserPrompts: Whether to enable user prompts (default is False)
          - userQuaternionControl: Whether to use quaternions for orientation control (default is False)
        '''

        self.enableUserPrompts = enableUserPrompts
        self.useQuaternionControl = useQuaternionControl

        if self.useQuaternionControl:
            NUM_ORIENTATION_DOFS = 4 # Orientation is defined using a w, x, y, z vector
        else:
            NUM_ORIENTATION_DOFS = 3 # Orientation is defined using a x, y, z vector

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

        if not self.useQuaternionControl:
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
        
        else:
            self.orientationUpdateMsg = QuatInterpMsg()

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

        if self.useQuaternionControl:
            self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualWorldOrientation", Float64MultiArray, self.rightOrientationTaskActualCallback)
        else:
            self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)

        self.rightOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandOrientation/errorAngle",    Float64,           self.rightOrientationTaskErrorCallback)

        if self.useQuaternionControl:
            self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualWorldOrientation", Float64MultiArray, self.leftOrientationTaskActualCallback)
        else:
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

        if not self.useQuaternionControl:
            self.rightOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/goalVector", Float64MultiArray, queue_size=1)

        self.rightOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/enabled", Int32, queue_size=1)
        self.rightOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/tare", Int32, queue_size=1)

        if not self.useQuaternionControl:
            self.leftOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/goalVector", Float64MultiArray, queue_size=1)

        self.leftOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/enabled", Int32, queue_size=1)
        self.leftOrientationTaskTarePublisher = rospy.Publisher("/dreamer_controller/LeftHandOrientation/tare", Int32, queue_size=1)


        if self.useQuaternionControl:
            self.orientationGoalPublisher = rospy.Publisher("/dreamer_controller/trajectory_generator/update_orientation", QuatInterpMsg, queue_size=1)


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
            self.rightOrientationTaskEnablePublisher.get_num_connections() == 0 or \
            self.leftCartesianTaskEnablePublisher.get_num_connections() == 0 or \
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
            self.currentRightCartesianPos == None or self.currentRightOrientation == None or \
            self.currentLeftCartesianPos == None or self.currentLeftOrientation == None):

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
        lhCartTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.lhCartWP)

        if not self.useQuaternionControl:
            rhOrientTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.rhOrientWP)
            lhOrientTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.lhOrientWP)

        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(traj.jPosWP)

        # Generate the trajectories
        rhCartTG.generateTrajectory(traj.duration)
        lhCartTG.generateTrajectory(traj.duration)

        if not self.useQuaternionControl:
            rhOrientTG.generateTrajectory(traj.duration)
            lhOrientTG.generateTrajectory(traj.duration)

        jPosTG.generateTrajectory(traj.duration)


        # ************************* new code to handle quaternion trajectories *****************************
        if self.useQuaternionControl:
            waypointTimesR = rhCartTG.getWaypointTimes();
            waypointTimesL = lhCartTG.getWaypointTimes();

            # print "right waypoint times: {0}".format(waypointTimesR)
            # print "left waypoint times: {0}".format(waypointTimesL)


        # *************************************************************************************************

        if self.enableUserPrompts:
            index = raw_input("Execute trajectory {0}? Y/n\n".format(traj.name))
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        print "DreamerInterface: followTrajectory: Following trajectory {0}...".format(traj.name)
        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= traj.duration:
                goalRightHandCartPos = rhCartTG.getLastPoint()
                goalLeftHandCartPos = lhCartTG.getLastPoint()
                if not self.useQuaternionControl:
                    goalRightHandOrientation = rhOrientTG.getLastPoint()
                    goalLeftHandOrientation = lhOrientTG.getLastPoint()
                else:
                    # The following settings should result in the final orientations being
                    # sent to ControlIt!
                    self.orientationUpdateMsg.rhStart.w = traj.getPenultimateRHOrient()[0]
                    self.orientationUpdateMsg.rhStart.x = traj.getPenultimateRHOrient()[1]
                    self.orientationUpdateMsg.rhStart.y = traj.getPenultimateRHOrient()[2]
                    self.orientationUpdateMsg.rhStart.z = traj.getPenultimateRHOrient()[3]

                    self.orientationUpdateMsg.lhStart.w = traj.getPenultimateLHOrient()[0]
                    self.orientationUpdateMsg.lhStart.x = traj.getPenultimateLHOrient()[1]
                    self.orientationUpdateMsg.lhStart.y = traj.getPenultimateLHOrient()[2]
                    self.orientationUpdateMsg.lhStart.z = traj.getPenultimateLHOrient()[3]

                    self.orientationUpdateMsg.rhEnd.w = traj.getFinalRHOrient()[0]
                    self.orientationUpdateMsg.rhEnd.x = traj.getFinalRHOrient()[1]
                    self.orientationUpdateMsg.rhEnd.y = traj.getFinalRHOrient()[2]
                    self.orientationUpdateMsg.rhEnd.z = traj.getFinalRHOrient()[3]

                    self.orientationUpdateMsg.lhEnd.w = traj.getFinalLHOrient()[0]
                    self.orientationUpdateMsg.lhEnd.x = traj.getFinalLHOrient()[1]
                    self.orientationUpdateMsg.lhEnd.y = traj.getFinalLHOrient()[2]
                    self.orientationUpdateMsg.lhEnd.z = traj.getFinalLHOrient()[3]

                    self.orientationUpdateMsg.rhProportion = 1
                    self.orientationUpdateMsg.lhProportion = 1

                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rhCartTG.getPoint(deltaTime)
                goalLeftHandCartPos = lhCartTG.getPoint(deltaTime)
                if not self.useQuaternionControl:
                    goalRightHandOrientation = rhOrientTG.getPoint(deltaTime)
                    goalLeftHandOrientation = lhOrientTG.getPoint(deltaTime)
                else:
                    # Here is where the magic happens...

                    # Determine which segment of the right hand orientation trajectory we are in...
                    startWPIndex = 0
                    for ii in range(len(waypointTimesR) - 1):
                        # print "DreamerInterface: followTrajectory: right proportion region search:\n"\
                        #       "  - waypoint times: {0}\n"\
                        #       "  - ii: {1}\n"\
                        #       "  - waypointTimesR[{1}]: {2}\n"\
                        #       "  - waypointTimesR[{3}]: {4}\n"\
                        #       "  - deltaTime: {5}".format(
                        #         waypointTimesR, ii, waypointTimesR[ii], ii+1, waypointTimesR[ii+1], deltaTime)

                        if (waypointTimesR[ii] <= deltaTime) and (waypointTimesR[ii + 1] >= deltaTime):
                            startWPIndex = ii
                            # print "*** Setting startWPIndex to be {0}\n".format(startWPIndex)

                    # Determine the total time of the right hand orientation segment we are in...
                    segmentTotalTime = waypointTimesR[startWPIndex + 1] - waypointTimesR[startWPIndex]
                    segmentDeltaTime = deltaTime - waypointTimesR[startWPIndex]

                    self.orientationUpdateMsg.rhStart.w = traj.rhOrientWP[startWPIndex][0]
                    self.orientationUpdateMsg.rhStart.x = traj.rhOrientWP[startWPIndex][1]
                    self.orientationUpdateMsg.rhStart.y = traj.rhOrientWP[startWPIndex][2]
                    self.orientationUpdateMsg.rhStart.z = traj.rhOrientWP[startWPIndex][3]

                    self.orientationUpdateMsg.rhEnd.w = traj.rhOrientWP[startWPIndex + 1][0]
                    self.orientationUpdateMsg.rhEnd.x = traj.rhOrientWP[startWPIndex + 1][1]
                    self.orientationUpdateMsg.rhEnd.y = traj.rhOrientWP[startWPIndex + 1][2]
                    self.orientationUpdateMsg.rhEnd.z = traj.rhOrientWP[startWPIndex + 1][3]
                    
                    self.orientationUpdateMsg.rhProportion = segmentDeltaTime / segmentTotalTime

                    # print "DreamerInterface: followTrajectory: right proportion computation:\n"\
                    #       "  - waypoint times: {0}\n"\
                    #       "  - deltaTime: {1}\n"\
                    #       "  - startWPIndex: {2}\n"\
                    #       "  - segmentTotalTime: {3}\n"\
                    #       "  - segmentDeltaTime: {4}\n"\
                    #       "  - proportion: {5}\n".format(
                    #         waypointTimesR, deltaTime, startWPIndex, segmentTotalTime,
                    #         segmentDeltaTime, self.orientationUpdateMsg.rhProportion)

                    for ii in range(len(waypointTimesL) - 1):
                        if waypointTimesL[ii] <= deltaTime and waypointTimesL[ii + 1] >= deltaTime:
                            startWPIndex = ii

                    # print "DreamerInterface: followTrajectory: left proportion region search:\n"\
                    #         "  - deltaTime: {0}\n"\
                    #         "  - waypoint times: {1}\n"\
                    #         "  - startWPIndex: {2}\n"\
                    #         "  - left orientation waypoints: {3}\n".format(
                    #             deltaTime, waypointTimesL, startWPIndex, traj.lhOrientWP)

                    # Determine the total time of the left hand orientation segment we are in...
                    segmentTotalTime = waypointTimesL[startWPIndex + 1] - waypointTimesL[startWPIndex]
                    segmentDeltaTime = deltaTime - waypointTimesL[startWPIndex]

                    self.orientationUpdateMsg.lhStart.w = traj.lhOrientWP[startWPIndex][0]
                    self.orientationUpdateMsg.lhStart.x = traj.lhOrientWP[startWPIndex][1]
                    self.orientationUpdateMsg.lhStart.y = traj.lhOrientWP[startWPIndex][2]
                    self.orientationUpdateMsg.lhStart.z = traj.lhOrientWP[startWPIndex][3]

                    self.orientationUpdateMsg.lhEnd.w = traj.lhOrientWP[startWPIndex + 1][0]
                    self.orientationUpdateMsg.lhEnd.x = traj.lhOrientWP[startWPIndex + 1][1]
                    self.orientationUpdateMsg.lhEnd.y = traj.lhOrientWP[startWPIndex + 1][2]
                    self.orientationUpdateMsg.lhEnd.z = traj.lhOrientWP[startWPIndex + 1][3]

                    self.orientationUpdateMsg.lhProportion = segmentDeltaTime / segmentTotalTime

                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            if not self.useQuaternionControl:
                self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
                self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            if not self.useQuaternionControl:
                self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
                self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            else:
                self.orientationGoalPublisher.publish(self.orientationUpdateMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        # print "Done following trajectory {0}!".format(traj.name)
        return not rospy.is_shutdown()

    def updateRightHandCartesianPosition(self, cartesianGoal):
        self.rightHandCartesianGoalMsg.data = cartesianGoal
        self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)

    def updateLeftHandCartesianPosition(self, cartesianGoal):
        self.leftHandCartesianGoalMsg.data = cartesianGoal
        self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)

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
        

    def closeLeftGripper(self):
        self.leftGripperCmdMsg.data = True # close the gripper
        self.leftGripperCmdPublisher.publish(self.leftGripperCmdMsg)

    def openLeftGripper(self):
        self.leftGripperCmdMsg.data = False # relax the gripper
        self.leftGripperCmdPublisher.publish(self.leftGripperCmdMsg)