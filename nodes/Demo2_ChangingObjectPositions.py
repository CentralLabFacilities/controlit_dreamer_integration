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

# PIPE_LOCATION = [0.28664480323526653, -0.1614844904659368, 0.9597645035426976]  # original
# PIPE_LOCATION = [0.33, -0.2, 0.9597645035426976]  # displacement 1
# PIPE_LOCATION = [0.38, 0.0, 0.9597645035426976]  # displacement 2
# PIPE_LOCATION = [0.2, -0.1, 0.9597645035426976]  # displacement 3
# PIPE_LOCATION = [0.38, -0.16, 1.2]  # displacement 4
# PIPE_LOCATION = [0.48, -0.16, 1.3]  # displacement 5
# PIPE_LOCATION = [0.0, -0.16, 1.15]  # displacement 6
# PIPE_LOCATION = [0.28, -0.3, 1.0]  # displacement 7
# PIPE_LOCATION = [0.28, -0.3, 1.2]  # displacement 8
PIPE_LOCATION = [0.35, -0.35, 1.1]  # displacement 9

class Demo2_ChangingObjectPositions:
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

        self.enableMsg = Int32()
        self.enableMsg.data = 1

        #-----------------------------------------------------------------------------'

        # Initialize member variables
        self.currentPosture = None
        self.postureError = None

        self.currentRightCartesianPos = None
        self.rightCartesianPosError = None

        self.currentRightOrientation = None
        self.rightOrientationError = None

        # Create the ROS topic subscriptions
        self.postureTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/JPosTask/actualPosition", Float64MultiArray, self.postureTaskActualCallback)
        self.postureTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/JPosTask/error",          Float64MultiArray, self.postureTaskErrorCallback)

        self.rightCartesianTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandPosition/actualWorldPosition", Float64MultiArray, self.rightCartesianTaskActualCallback)
        self.rightCartesianTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandPosition/error",               Float64MultiArray, self.rightCartesianTaskErrorCallback)

        self.rightOrientationTaskActualSubscriber = rospy.Subscriber("/dreamer_controller/RightHandOrientation/actualHeading", Float64MultiArray, self.rightOrientationTaskActualCallback)
        self.rightOrientationTaskErrorSubscriber  = rospy.Subscriber("/dreamer_controller/RightHandOrientation/errorAngle",    Float64,           self.rightOrientationTaskErrorCallback)

        # Create the ROS topic publishers
        self.postureTaskGoalPublisher = rospy.Publisher("/dreamer_controller/JPosTask/goalPosition", Float64MultiArray, queue_size=1)
        
        self.rightCartesianTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/goalPosition", Float64MultiArray, queue_size=1)
        self.rightCartesianTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandPosition/enabled", Int32, queue_size=1)
        
        self.rightOrientationTaskGoalPublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/goalVector", Float64MultiArray, queue_size=1)
        self.rightOrientationTaskEnablePublisher = rospy.Publisher("/dreamer_controller/RightHandOrientation/enabled", Int32, queue_size=1)
        
        self.rightHandCmdPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/powerGrasp", Bool, queue_size=1)
        self.selectIndexFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightIndexFinger", Bool, queue_size=1)
        self.selectMiddleFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightMiddleFinger", Bool, queue_size=1)
        self.selectPinkyFingerPublisher = rospy.Publisher("/dreamer_controller/controlit/rightHand/includeRightPinkyFinger", Bool, queue_size=1)


    def postureTaskActualCallback(self, msg):
        self.currentPosture = msg.data

    def postureTaskErrorCallback(self, msg):
        self.postureError = msg.data

    def rightCartesianTaskActualCallback(self, msg):
        self.currentRightCartesianPos = msg.data

    def rightCartesianTaskErrorCallback(self, msg):
        self.rightCartesianPosError = msg.data

    def rightOrientationTaskActualCallback(self, msg):
        self.currentRightOrientation = msg.data

    def rightOrientationTaskErrorCallback(self, msg):
        self.rightOrientationError = msg.data

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
            self.rightOrientationTaskEnablePublisher.get_num_connections() == 0):

            if warningPrinted:
                if self.rightCartesianTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on right cartesian enable subscriber..."
                if self.rightOrientationTaskEnablePublisher.get_num_connections() == 0:
                    print "Waiting on right orientation enable subscriber..."
                    
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
        self.rightOrientationTaskEnablePublisher.publish(enableMsg)

        # Initialize the default posture
        self.postureGoalMsg.data = DEFAULT_POSTURE
        self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

        # Wait for connection to ControlIt!
        pauseCount = 0
        warningPrinted = False
        while not rospy.is_shutdown() and (
            self.currentPosture == None or \
            self.currentRightCartesianPos == None or self.currentRightOrientation == None):

            if warningPrinted:
                if self.currentRightCartesianPos == None:
                    print "Waiting on right hand position state..."
                if self.currentRightOrientation == None:
                    print "Waiting on right hand orientation state..."
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
        jPosWP = []

        # These are the initial values as specified in the YAML ControlIt! configuration file
        rightHandCartesianWP.append([0.033912978219317776, -0.29726881641499886, 0.82])
        rightHandOrientationWP.append([1.0, 0.0, 0.0])
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

        jPosWP.append([0.06826499288341317, 0.06826499288341317, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])
        jPosWP.append([0.0686363596318602,  0.0686363596318602,  
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        jPosWP.append([0.06804075180539401, 0.06804075180539401, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        jPosWP.append([0.06818415549992426, 0.06818415549992426, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        jPosWP.append([0.06794500584573498, 0.06794500584573498, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
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
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
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
        jPosWP = []

        originalCartesianPoint = [0.25822435038901964, -0.1895604971725577, 1.0461857180093073]

        # This is the last configuration of the gotToReady trajectory
        rightHandCartesianWP.append(originalCartesianPoint)
        # rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        # jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # linearly interpolate four points between the original point and the pipe's location
        for ii in range(4):
            step = ii + 1.0
            xx = (PIPE_LOCATION[0] - originalCartesianPoint[0]) / 4.0 * step + originalCartesianPoint[0]
            yy = (PIPE_LOCATION[1] - originalCartesianPoint[1]) / 4.0 * step + originalCartesianPoint[1]
            zz = (PIPE_LOCATION[2] - originalCartesianPoint[2]) / 4.0 * step + originalCartesianPoint[2]
            cartesianPos = []
            cartesianPos.append(xx)
            cartesianPos.append(yy)
            cartesianPos.append(zz)
            rightHandCartesianWP.append(cartesianPos)
        

        # Create the WayPoints
        # rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        # rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        # rightHandCartesianWP.append([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        # rightHandCartesianWP.append([0.2926328098812538, -0.22212659727858264, 0.9685956358633105])
        # rightHandCartesianWP.append([0.28750632029946943, -0.17027266952524717, 0.9597899484960192])
        # rightHandCartesianWP.append([0.28664480323526653, -0.1614844904659368, 0.9597645035426976])
        rightHandCartesianWP.append(PIPE_LOCATION) # displacement 1

        # rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        # rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        # rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        # rightHandOrientationWP.append([0.6030193813610835, -0.6721560435204502, 0.42962062201648427])
        # rightHandOrientationWP.append([0.6847262101203426, -0.7283419844816242, 0.025845131371348223])
        # rightHandOrientationWP.append([0.8206856971797751, -0.5665754546656667, -0.0739407912788299])
        # rightHandOrientationWP.append([0.830926574184253, -0.5512666962638427, -0.07527322169782114])
       
        # jPosWP.append([0.09594703765058178, 0.09594703765058178, 
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                0.09817730939874109, 0.1020579374634571, 0.0836978735049272, 1.6235470575907778, 1.054005683347489, -0.6934016966989962, -0.4214573788290379])
        # jPosWP.append([0.09578187031551673, 0.09578187031551673, 
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                0.07827516108651086, 0.06968225019914681, -0.0651398024593994, 1.3456921412703295, 1.3295641014614135, -0.6445024104856519, -0.4748814187628949])
        # jPosWP.append([0.09578234092970726, 0.09578234092970726, 
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                0.09157211596703656, 0.041108271773515705, -0.22384970463739684, 1.3076704033792463, 1.353903257753508, -0.7185241326180924, -0.454150460888528])
        # jPosWP.append([0.09590536736161434, 0.09590536736161434, 
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        # rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 5.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # jPosTG.generateTrajectory(TOTAL_TRAVEL_TIME)

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
                # goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                # goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            # self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            # self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz


        doPowerGrasp = True
        if ENABLE_USER_PROMPTS:
            index = raw_input("Perform right hand power grasp? Y/n\n")
            if index == "N" or index == "n":
                doPowerGrasp = False
        
        if doPowerGrasp:
            # Exclude index finger from power grasp
            # self.rightIndexFingerCmdMsg.data = False
            # self.selectIndexFingerPublisher.publish(self.rightIndexFingerCmdMsg)

            # Exclude middle finger from power grasp
            # self.rightMiddleFingerCmdMsg.data = False
            # self.selectMiddleFingerPublisher.publish(self.rightMiddleFingerCmdMsg)

            self.rightHandCmdMsg.data = True
            self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)

        # print "Done grabbing metal tube!"
        return not rospy.is_shutdown()

    def goToIdlePosition(self):

        # Define the waypoints
        # Note waypoints are formatted as: [[x, y, z], ...]

        rightHandCartesianWP = []
        rightHandOrientationWP = []
        jPosWP = []


        # This is the last configuration of the grabMetalTube trajectory
        rightHandCartesianWP.append(PIPE_LOCATION)
        rightHandOrientationWP.append([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])        
        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # This is the last configuration of the grabMetalTube trajectory
        # rightHandCartesianWP.append(PIPE_LOCATION)
        # rightHandOrientationWP.append([0.830926574184253, -0.5512666962638427, -0.07527322169782114])
        # jPosWP.append([0.09590536736161434, 0.09590536736161434, 
        #                0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
        #                0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

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

        jPosWP.append([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm
        jPosWP.append([0.06794500584573498, 0.06794500584573498, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        jPosWP.append([0.06818415549992426, 0.06818415549992426, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        jPosWP.append([0.06804075180539401, 0.06804075180539401, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        jPosWP.append([0.0686363596318602,  0.0686363596318602,  
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        jPosWP.append([0.06826499288341317, 0.06826499288341317, 
                       0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,     # left arm
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])        

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        jPosTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(jPosWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
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
                goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
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

        print "Done going to idle position!"
        return True

    def run(self):
        """
        Runs the demo 2 behavior.
        """

        if not self.connectToControlIt():
            return

        index = raw_input("Start demo 2? Y/n\n")
        if index == "N" or index == "n":
            return

        if not self.goToReadyPosition():
            return

        if not self.grabMetalTube():
            return

        if not self.goToIdlePosition():
            return

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo2_ChangingObjectPositions', anonymous=True)

    demo = Demo2_ChangingObjectPositions()
    # t = threading.Thread(target=demo.run)
    # t.start()
    demo.run()

    print "Demo 2 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting