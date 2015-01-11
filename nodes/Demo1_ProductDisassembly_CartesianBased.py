#!/usr/bin/env python

'''
This uses both posture and orientation control.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = True

NUM_CARTESIAN_DOFS = 3 # Cartesian goal is x, y, z
NUM_ORIENTATION_DOFS = 3 # Orientation is defined using a x, y, z vector
NUM_ROBOT_DOFS = 16

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

class Demo1_ProductDisassembly:
    def __init__(self):

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
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/JPosTask/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.postureTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/JPosTask/error",          Float64MultiArray, self.postureTaskErrorCallback)

        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.rightCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandPosition/error",               Float64MultiArray, self.rightCartesianTaskErrorCallback)

        self.leftCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandPosition/actualWorldPosition",  Float64MultiArray, self.leftCartesianTaskActualCallback)
        self.leftCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandPosition/error",                Float64MultiArray, self.leftCartesianTaskErrorCallback)

        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.rightOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandOrientation/errorAngle",    Float64,           self.rightOrientationTaskErrorCallback)

        self.leftOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/actualHeading", Float64MultiArray, self.leftOrientationTaskActualCallback)
        self.leftOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/LeftHandOrientation/errorAngle",    Float64,           self.leftOrientationTaskErrorCallback)

        # Create the ROS topic publishers
        self.postureTaskGoalPublisher = rospy.Publisher("/dreamer_controller/JPosTask/goalPosition", Float64MultiArray, queue_size=1)
        self.postureTaskTarePublisher = rospy.Publisher("/dreamer_controller/JPosTask/tare", Int32, queue_size=1)

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

    def connectToControlIt(self):
        print "Connecting to ControlIt!..."

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
            return

        # Enable the Cartesian position and orientation tasks
        enableMsg = Int32()
        enableMsg.data = 1        
        self.rightCartesianTaskEnablePublisher.publish(enableMsg)
        self.leftCartesianTaskEnablePublisher.publish(enableMsg)
        self.rightOrientationTaskEnablePublisher.publish(enableMsg)
        self.leftOrientationTaskEnablePublisher.publish(enableMsg)

        # Initialize the default posture
        self.postureGoalMsg.data = DEFAULT_POSTURE
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
            return

        print "Done connecting to ControlIt!"
        return not rospy.is_shutdown()

    def goToReadyPosition(self):

        # print "Generating GoToReady trajectories..."

        # Define the waypoints
        # Note waypoints are formatted as: [[x, y, z], ...]

        rightHandCartesianWP = []
        rightHandOrientationWP = []
        leftHandCartesianWP = []
        leftHandOrientationWP = []
        jPosWP = []

        # These are the initial values as specified in the YAML ControlIt! configuration file
        rightHandCartesianWP.append([0.033912978219317776, -0.29726881641499886, 0.82])
        rightHandOrientationWP.append([1.0, 0.0, 0.0])
        leftHandCartesianWP.append([0.033912978219317776, 0.29726881641499886, 0.82])
        leftHandOrientationWP.append([1.0, 0.0, 0.0])
        jPosWP.append(DEFAULT_POSTURE)
        
        # 2015.01.06 Trajectory
        rightHandCartesianWP.append([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])
        rightHandCartesianWP.append([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        rightHandCartesianWP.append([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        rightHandCartesianWP.append([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        rightHandCartesianWP.append([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])

        rightHandOrientationWP.append([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        rightHandOrientationWP.append([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        rightHandOrientationWP.append([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        rightHandOrientationWP.append([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        rightHandOrientationWP.append([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])

        leftHandCartesianWP.append([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        leftHandCartesianWP.append([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        leftHandCartesianWP.append([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        leftHandCartesianWP.append([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        leftHandCartesianWP.append([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        leftHandCartesianWP.append([0.25822435038901964, 0.24,               1.0461857180093073])

        leftHandOrientationWP.append([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])
        leftHandOrientationWP.append([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        leftHandOrientationWP.append([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        leftHandOrientationWP.append([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        leftHandOrientationWP.append([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])

        jPosWP.append([0.06826499288341317, 0.06826499288341317, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])
        jPosWP.append([0.0686363596318602,  0.0686363596318602,  
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, 
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        jPosWP.append([0.06804075180539401, 0.06804075180539401, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        jPosWP.append([0.06818415549992426, 0.06818415549992426, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        jPosWP.append([0.06794500584573498, 0.06794500584573498, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Go ready position? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
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

        # print "Done going to ready position!"
        return not rospy.is_shutdown()

    def grabMetalTube(self):
        """
        Executes the trajectory that moves the right hand to be around the metal tube.
        """

        rightHandCartesianWP = []
        rightHandOrientationWP = []
        leftHandCartesianWP = []
        leftHandOrientationWP = []
        jPosWP = []

        # Left hand does not move
        leftHandPosition = [0.25822435038901964, 0.24, 1.0461857180093073]
        leftHandOrientation = [0.5409881394605172, 0.8191390472602035, 0.19063854336595773]

        # This is the last configuration of the gotToReady trajectory
        rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        leftHandCartesianWP.append(leftHandPosition)
        leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # Create the WayPoints
        rightHandCartesianWP.append([0.3137686413286708, -0.26659865390759796, 1.0541700108404664])
        rightHandCartesianWP.append([0.2926328098812538, -0.22212659727858264, 0.9685956358633105])
        rightHandCartesianWP.append([0.28750632029946943, -0.17027266952524717, 0.9597899484960192])
        rightHandCartesianWP.append([0.28664480323526653, -0.1614844904659368, 0.9597645035426976])

        rightHandOrientationWP.append([0.6030193813610835, -0.6721560435204502, 0.42962062201648427])
        rightHandOrientationWP.append([0.6847262101203426, -0.7283419844816242, 0.025845131371348223])
        rightHandOrientationWP.append([0.8206856971797751, -0.5665754546656667, -0.0739407912788299])
        rightHandOrientationWP.append([0.830926574184253, -0.5512666962638427, -0.07527322169782114])

        leftHandCartesianWP.append(leftHandPosition)
        leftHandCartesianWP.append(leftHandPosition)
        leftHandCartesianWP.append(leftHandPosition)
        leftHandCartesianWP.append(leftHandPosition)

        leftHandOrientationWP.append(leftHandOrientation)
        leftHandOrientationWP.append(leftHandOrientation)
        leftHandOrientationWP.append(leftHandOrientation)
        leftHandOrientationWP.append(leftHandOrientation)

       
        jPosWP.append([0.09594703765058178, 0.09594703765058178, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09817730939874109, 0.1020579374634571, 0.0836978735049272, 1.6235470575907778, 1.054005683347489, -0.6934016966989962, -0.4214573788290379])
        jPosWP.append([0.09578187031551673, 0.09578187031551673, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.07827516108651086, 0.06968225019914681, -0.0651398024593994, 1.3456921412703295, 1.3295641014614135, -0.6445024104856519, -0.4748814187628949])
        jPosWP.append([0.09578234092970726, 0.09578234092970726, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09157211596703656, 0.041108271773515705, -0.22384970463739684, 1.3076704033792463, 1.353903257753508, -0.7185241326180924, -0.454150460888528])
        jPosWP.append([0.09590536736161434, 0.09590536736161434, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Position right hand around metal tube? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
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


        doPowerGrasp = True
        if ENABLE_USER_PROMPTS:
            index = raw_input("Perform right hand power grasp? Y/n\n")
            if index == "N" or index == "n":
                doPowerGrasp = False
        
        if doPowerGrasp:
            # Exclude index finger from power grasp
            self.rightIndexFingerCmdMsg.data = False
            self.selectIndexFingerPublisher.publish(self.rightHandCmdMsg)

            self.rightHandCmdMsg.data = True
            self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)

        # print "Done grabbing metal object!"
        return not rospy.is_shutdown()

    def grabRubberValve(self):
        """
        Executes the trajectory that moves the left hand to be around a position where it can grab the rubber object.
        """

        rightHandCartesianWP = []
        rightHandOrientationWP = []
        leftHandCartesianWP = []
        leftHandOrientationWP = []
        jPosWP = []

        # Right hand does not move
        rightHandPosition = [0.28664480323526653, -0.1614844904659368, 0.9597645035426976]
        rightHandOrientation = [0.830926574184253, -0.5512666962638427, -0.07527322169782114]

        # This is the last configuration of the grabMetalTube trajectory
        rightHandCartesianWP.append(rightHandPosition)
        rightHandOrientationWP.append(rightHandOrientation)
        leftHandCartesianWP.append([0.25822435038901964, 0.24, 1.0461857180093073])
        leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        jPosWP.append([0.09590536736161434, 0.09590536736161434, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        rightHandCartesianWP.append(rightHandPosition)
        rightHandCartesianWP.append(rightHandPosition)
        rightHandCartesianWP.append(rightHandPosition)
        rightHandCartesianWP.append(rightHandPosition)

        rightHandOrientationWP.append(rightHandOrientation)
        rightHandOrientationWP.append(rightHandOrientation)
        rightHandOrientationWP.append(rightHandOrientation)
        rightHandOrientationWP.append(rightHandOrientation)

        leftHandCartesianWP.append([0.2558247304604975, 0.2949244102085959, 1.1042523046819483])
        leftHandCartesianWP.append([0.24986753716571164, 0.2597244717600235, 1.075447863944082])
        leftHandCartesianWP.append([0.29308174503554474, 0.24554806065234056, 1.0515318016118187])
        leftHandCartesianWP.append([0.2808559231427563, 0.2393468245212046, 1.0249428672824978])
        
        leftHandOrientationWP.append([-0.06355664672818866, -0.3507624169373639, 0.9343052389454567])
        leftHandOrientationWP.append([-0.05743278073528967, -0.24859633215529697, 0.9669029627299452])
        leftHandOrientationWP.append([-0.11088559335760977, -0.1963599862969318, 0.9742418287915992])
        leftHandOrientationWP.append([-0.0688185464892449, -0.10737620829573602, 0.9918338356555186])

        jPosWP.append([0.09590536736161434, 0.09590536736161434, 
            -0.15114015512345808, 0.6757263833050331, -0.1512750672164876, 1.9078665565670625, 0.20486719263640862, 0.09583174147398679, -0.5754068224606331, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        jPosWP.append([0.09590536736161434, 0.09590536736161434,  
            -0.13966183280088498, 0.4723217587608189, -0.17997921407313958, 1.8670358353680174, 0.1170707192657332, 0.11627257977324582, -0.6470561008686319, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        jPosWP.append([0.09590536736161434, 0.09590536736161434,  
            0.03507646845214241, 0.32010870159025234, -0.18088026181707248, 1.6732313049447447, 0.06210729345798295, 0.1827444704387443, -0.9045072348249086, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        jPosWP.append([0.09590536736161434, 0.09590536736161434, 
            0.002794296425643595, 0.2835844757183357, -0.18807469031404708, 1.6100747424098305, 0.11839413877386204, 0.248941084731985, -0.9437127120955863, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Position left hand around rubber valve? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
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

        performGrasp = True
        if ENABLE_USER_PROMPTS:
            index = raw_input("Perform left gripper power grasp? Y/n\n")
            if index == "N" or index == "n":
                performGrasp = False
        
        if performGrasp:
            self.leftGripperCmdMsg.data = True
            self.leftGripperCmdPublisher.publish(self.leftGripperCmdMsg)

        print "Done grabbing rubber object!"
        return not rospy.is_shutdown()

    def liftRubberObject(self):
        """
        Executes the trajectory lifts the rubber object and positions it over the left box.
        """

        jPosWP = []

        # This is the last configuration of the grabRubberValve trajectory
        jPosWP.append([0.10548164996775973, 0.10548164996775973, 
                       -0.16854400148191434, 0.4167463016338474, -0.2701796881433479, 1.83259571, 0.11149895105568859, 0.2956444253241622, -0.7240388727126355,            # left arm
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        jPosWP.append([0.10580113260054594, 0.10580113260054594, 
                       0.23979689653764183, 0.5366671561387937, -0.24028499984085533, 1.9690385406401882, 0.4610325861244941, -0.01797598419331518, -1.0919871103215044, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.105459767394544, 0.105459767394544, 
                       0.6906767499136286, 0.41385749135856686, -0.03285041562768612, 1.640803065788199, 0.52732519608718, -0.01748961748122057, -1.0923600305447438, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.10545244652292243, 0.10545244652292243, 
                       1.011134048992465, 0.37656249916293216, 0.06403932455903373, 1.2390040299773695, -0.2878908639092968, -0.4295398542143753, -0.7719594438307211, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.1059528459107981, 0.1059528459107981, 
                       1.1823048236038691, 0.4227435883156559, 0.03803891004078773, 0.6072615761362916, -0.32622157205509805, -0.39518622920297136, -0.7588162298258935, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        
        # Create the trajectory generators
        # rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        # rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        # leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        # rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Lift rubber object? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            # goalRightHandCartPos = None
            # goalRightHandOrientation = None
            # goalJPos = None

            if deltaTime >= TOTAL_TRAVEL_TIME:
                # goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                # goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                # goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                # goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                # goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                # goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            # self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            # self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            # self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            # self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            # self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            # self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        releaseGripper = True
        if ENABLE_USER_PROMPTS:
            index = raw_input("Release left gripper power grasp? Y/n\n")
            if index == "N" or index == "n":
                releaseGripper = False

        if releaseGripper:
            self.leftGripperCmdMsg.data = False  # relax grasp
            self.leftGripperCmdPublisher.publish(self.leftGripperCmdMsg)

        addRightIndexFinger = True
        if ENABLE_USER_PROMPTS:
            index = raw_input("Add right index finger to power grasp? Y/n\n")
            if not (index == "N" or index == "n"):
                addRightIndexFinger = False

        if addRightIndexFinger:
            self.rightIndexFingerCmdMsg.data = True
            self.selectIndexFingerPublisher.publish(self.rightHandCmdMsg)

        print "Done lifting rubber object!"
        return not rospy.is_shutdown()

    def liftLeftArmOutOfBox(self):
        """
        Executes the trajectory lifting the left arm out of the box
        """

        jPosWP = []

        # This is the last configuration of the liftRubberObject trajectory
        jPosWP.append([0.1059528459107981, 0.1059528459107981, 
                       1.1823048236038691, 0.4227435883156559, 0.03803891004078773, 0.6072615761362916, -0.32622157205509805, -0.39518622920297136, -0.7588162298258935, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        jPosWP.append([0.10619477867763813, 0.10619477867763813, 
                       1.2202454432859893, 0.05738140848539306, 0.3189522271170663, 0.7232510537980966, -0.1726463028864427, -0.6028618209841994, 0.24500111186631893, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.10643665665219065, 0.10643665665219065, 
                       1.5285064325031241, 0.12098014955475167, 0.29442826231308566, 0.9381167644779234, -0.33959274258259425, -0.9185511712204342, 0.10626589612738678, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.10626453436778832, 0.10626453436778832, 
                       0.661416424392025, 0.1485420097610339, 0.3287541049538422, 1.8674898940472466, -0.13103595620300554, -0.9935935620370674, -0.21511362204881762, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.10583243639915946, 0.10583243639915946, 
                       0.11249207204916184, 0.10479901770041052, 0.3332522844950901, 2.3037089105630066, -0.20471605919890376, -0.9940708398968128, -0.3071054312567586, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        jPosWP.append([0.1059981788687132, 0.1059981788687132,
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        
        # Create the trajectory generators
        # rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        # rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        # leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        # rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Lift left arm out of box? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            # goalRightHandCartPos = None
            # goalRightHandOrientation = None
            # goalJPos = None

            if deltaTime >= TOTAL_TRAVEL_TIME:
                # goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                # goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                # goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                # goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                # goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                # goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            # self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            # self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            # self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            # self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            # self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            # self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        print "Done lifting left arm out of box!"
        return not rospy.is_shutdown()

    def placeTubeInBox(self):
        """
        Executes the trajectory that puts the metal tube in the box.
        """

        jPosWP = []

        # This is the last configuration of the liftLeftArmOutOfBox trajectory
        jPosWP.append([0.1059981788687132, 0.1059981788687132,
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])  # right arm

        jPosWP.append([0.10432430565653919, 0.10432430565653919, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.034478264115051754, 0.050983853806045615, 0.08104904608448296, 1.8817247736502365, 0.9521803557070976, -0.6771010281517622, -0.4043295777519238]) # right arm
        jPosWP.append([0.10412618474968699, 0.10412618474968699, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.02640397181637044, -0.03708730110774295, 0.5450790880102646, 2.141813410250169, 0.59083630516349, -0.7125783320482355, -0.672996092257354])       # right arm
        jPosWP.append([0.10495304042092034, 0.10495304042092034, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.5235748843032342, 0.027144300951222916, 0.4489218538612669, 1.8577598398066606, 0.2786073716961595, -0.7205390588078509, -0.6691257351559342])     # right arm
        jPosWP.append([0.10529825572993265, 0.10529825572993265, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.893718014139769, 0.022189852043952615, 0.25391928553092946, 1.4869481709613361, -0.04994059069942265, -0.624367239720437, -0.4343711477894259])    # right arm
        jPosWP.append([0.10504165418630391, 0.10504165418630391, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       1.1246908322214502, 0.04338531905479525, 0.260906295518494, 1.0195136153943856, -0.20861493889703248, -0.4437284194541821, -0.4310428017453589])     # right arm

        
        # Create the trajectory generators
        # rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        # rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        # leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        # rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Lift metal tube and place it in the box? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            # goalRightHandCartPos = None
            # goalRightHandOrientation = None
            # goalJPos = None

            if deltaTime >= TOTAL_TRAVEL_TIME:
                # goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                # goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                # goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                # goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                # goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                # goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            # self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            # self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            # self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            # self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            # self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            # self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        releasePowerGrasp = True

        if ENABLE_USER_PROMPTS:
            index = raw_input("Release right hand power grasp? Y/n\n")
            if index == "N" or index == "n":
                releasePowerGrasp = False

        if releasePowerGrasp:
            self.rightHandCmdMsg.data = False  # relax grasp
            self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)

        print "Done placing tube in box!"
        return not rospy.is_shutdown()

    def removeRightHandFromBox(self):
        """
        Executes the trajectory that removes the right hand from the box.
        """

        jPosWP = []

        # This is the last configuration of the placeTubeInBox trajectory
        jPosWP.append([0.10504165418630391, 0.10504165418630391, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       1.1246908322214502, 0.04338531905479525, 0.260906295518494, 1.0195136153943856, -0.20861493889703248, -0.4437284194541821, -0.4310428017453589])     # right arm

        jPosWP.append([0.10569569813223582, 0.10569569813223582, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       1.2163328893732905, 0.22569420623412806, 0.12048969643886341, 1.0417177826656587, -0.007464719474735143, -0.5137164939035933, -0.3957809598196772])  # right arm
        jPosWP.append([0.10574401313049778, 0.10574401313049778, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.7397335721725239, 0.14117852326765132, 0.23724265240496786, 1.6944240945727442, 0.6944317983570832, -0.6549455740031402, -0.40816321974940045])     # right arm
        jPosWP.append([0.10439315309874873, 0.10439315309874873, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.16597850687837903, 0.21017650186148398, 0.06531500696244147, 1.9265909647670418, 0.8620543726577191, -0.4644505843758746, -0.6566952560811945])    # right arm
        jPosWP.append([0.10425184299714459, 0.10425184299714459, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.11847755416302001, 0.06422356595783688, 0.327169061878712, 1.8993781491277486, 1.3082667536728438, -0.9510221581678175, -0.45723182354673425])   # right arm
        
        # Create the trajectory generators
        # rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        # rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        # leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        # rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Remove right hand from box? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            # goalRightHandCartPos = None
            # goalRightHandOrientation = None
            # goalJPos = None

            if deltaTime >= TOTAL_TRAVEL_TIME:
                # goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                # goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                # goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                # goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                # goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                # goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            # self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            # self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            # self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            # self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            # self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            # self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        print "Done taking right hand out of box!"
        return not rospy.is_shutdown()

    def goToIdlePosition(self):

        # Define the waypoints
        # Note waypoints are formatted as: [[x, y, z], ...]

        rightHandCartesianWP = []
        rightHandOrientationWP = []
        leftHandCartesianWP = []
        leftHandOrientationWP = []
        jPosWP = []


        # This is the last configuration of the gotToReady trajectory
        # rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        # rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        # leftHandCartesianWP.append([0.25822435038901964, 0.1895604971725577, 1.0461857180093073])
        # leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        # jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
        #                -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
        #                -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # This is the last configuration of the grabMetalTube trajectory
        # rightHandCartesianWP.append([0.28664480323526653, -0.1614844904659368, 0.9597645035426976])
        # rightHandOrientationWP.append([0.830926574184253, -0.5512666962638427, -0.07527322169782114])
        # leftHandCartesianWP.append([0.25822435038901964, 0.1895604971725577, 1.0461857180093073])
        # leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        # jPosWP.append([0.09590536736161434, 0.09590536736161434, 
        #                -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
        #                0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # This is the last configuration of the grabRubberValve trajectory
        rightHandCartesianWP.append([0.28664480323526653, -0.1614844904659368, 0.9597645035426976])
        rightHandOrientationWP.append([0.830926574184253, -0.5512666962638427, -0.07527322169782114])
        leftHandCartesianWP.append([0.2808559231427563, 0.2393468245212046, 1.0249428672824978])
        leftHandOrientationWP.append([-0.0688185464892449, -0.10737620829573602, 0.9918338356555186])
        jPosWP.append([0.09590536736161434, 0.09590536736161434, 
            0.002794296425643595, 0.2835844757183357, -0.18807469031404708, 1.6100747424098305, 0.11839413877386204, 0.248941084731985, -0.9437127120955863, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        
        # 2015.01.06 Trajectory
        rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        rightHandCartesianWP.append([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        rightHandCartesianWP.append([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        rightHandCartesianWP.append([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        rightHandCartesianWP.append([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        rightHandCartesianWP.append([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])

        rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        rightHandOrientationWP.append([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        rightHandOrientationWP.append([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        rightHandOrientationWP.append([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        rightHandOrientationWP.append([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        rightHandOrientationWP.append([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        
        leftHandCartesianWP.append([0.25822435038901964, 0.1895604971725577, 1.0461857180093073])
        leftHandCartesianWP.append([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        leftHandCartesianWP.append([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        leftHandCartesianWP.append([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        leftHandCartesianWP.append([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        leftHandCartesianWP.append([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        
        leftHandOrientationWP.append([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        leftHandOrientationWP.append([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        leftHandOrientationWP.append([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        leftHandOrientationWP.append([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        leftHandOrientationWP.append([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        leftHandOrientationWP.append([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])

        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       -0.08569654146540764, 0.07021124925432169,                    0, 1.7194162945362514, 1.51, -0.07, -0.18,    # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm
        jPosWP.append([0.06794500584573498, 0.06794500584573498, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        jPosWP.append([0.06818415549992426, 0.06818415549992426, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        jPosWP.append([0.06804075180539401, 0.06804075180539401, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        jPosWP.append([0.0686363596318602,  0.0686363596318602,  -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        jPosWP.append([0.06826499288341317, 0.06826499288341317, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])        

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        if ENABLE_USER_PROMPTS:
            index = raw_input("Go idle position? Y/n\n")
            if index == "N" or index == "n":
                return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            if deltaTime >= TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
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

        print "Done going to idle position!"
        return True

    def run(self):
        """
        Runs the Cartesian and orientation demo 1 behavior.
        """

        if not self.connectToControlIt():
            return

        index = raw_input("Start demo? Y/n\n")
        if index == "N" or index == "n":
            return

        # if not self.doTare():
        #     return

        if not self.goToReadyPosition():
            return

        if not self.grabMetalTube():
            return

        if not self.grabRubberValve():
            return

        # if not self.liftRubberObject():
        #     return

        # if not self.liftLeftArmOutOfBox():
        #     return

        # if not self.placeTubeInBox():
        #     return

        # if not self.removeRightHandFromBox():
        #     return

        # if rospy.is_shutdown():
        #     return False

        # if rospy.is_shutdown():
        #     return False

        if not self.goToIdlePosition():
            return

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo1_ProductDisassembly_CartesianBased', anonymous=True)

    demo = Demo1_ProductDisassembly()
    # t = threading.Thread(target=demo.run)
    # t.start()
    demo.run()

    print "Demo 1 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting