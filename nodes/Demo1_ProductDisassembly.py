#!/usr/bin/env python

'''
Implements the Demo 1 of ControlIt! and Dreamer disassembling a part.
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

NUM_CARTESIAN_DOFS = 3 # Cartesian goal is x, y, z
NUM_ORIENTATION_DOFS = 3 # Orientation is defined using a x, y, z vector
NUM_ROBOT_DOFS = 16

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

    # def linearInterpolate(self, trajectory, deltaTime):
    #     """
    #     Computes the position along the trajectory at the specified deltaTime.
    #     The trajectory is assumed to have millisecond resolution.
    #     deltaTime is assumed to be in seconds.
    #     """
    #     lowerIndex = math.floor(deltaTime * 1000)
    #     if lowerIndex >= len(trajectory):
    #         print "linearInterpolate: WARNING: deltaTime is beyond end of trajectory!\n"\
    #               "  - length of trajectory: {0}\n"\
    #               "  - delta time: {1}\n"\
    #               "Returning last position in trajectory.".format(len(trajectory), deltaTime)
    #         return trajectory[len(trajectory) - 1]
    #     elif lowerIndex == len(trajectory) - 1:
    #         return trajectory[len(trajectory) - 1]
    #     else:
    #         beforePoint = trajectory[lowerIndex]
    #         afterPoint = trajectory[lowerIndex + 1]

    #         # compute fraction of milliseconds that have elapsed
    #         fractionElapsed = (deltaTime * 1000) - math.floor(deltaTime * 1000)

    #         # do linear interpolation
    #         return (afterPoint - beforePoint) * fractionElapsed + beforePoint

    def getTimeSeconds(self):
        """
        Returns the current time in seconds.
        """
        return rospy.get_time()

    def issueTareCommands(self):
        print "Issuing tare commands..."
        self.postureTaskTarePublisher.publish(self.tareMsg)
        self.rightCartesianTaskTarePublisher.publish(self.tareMsg)
        self.leftCartesianTaskTarePublisher.publish(self.tareMsg)
        self.rightOrientationTaskTarePublisher.publish(self.tareMsg)
        self.leftOrientationTaskTarePublisher.publish(self.tareMsg)

        time.sleep(1)

        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and self.postureError == None:
        # while not rospy.is_shutdown() and (self.postureError == None or \
        #       self.rightCartesianPosError == None or self.leftCartesianPosError == None or \
        #       self.rightOrientationError == None or self.leftOrientationError == None):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for error information..."
                printWarning = True

        return not rospy.is_shutdown()

    def doTare(self):

        # Wait for connection to ControlIt!
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and self.postureTaskTarePublisher.get_num_connections() == 0:
        # while not rospy.is_shutdown() and (self.postureTaskTarePublisher.get_num_connections() == 0 or \
        #       self.rightCartesianTaskTarePublisher.get_num_connections() == 0 or \
        #       self.leftCartesianTaskTarePublisher.get_num_connections() == 0):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for connection to ControlIt!..."
                printWarning = True

        if rospy.is_shutdown():
            return False

        if not self.issueTareCommands():
            return False
        
        done = False
        while not done:

            resultList = [["Index", "Joint Name", "Error (rad)", "Error (deg)"]]
            resultList.append(["-----", "----------", "-----------", "-----------"])
            
            JOINT_NAMES = ["torso_lower_pitch", "torso_upper_pitch",
                           "left_shoulder_extensor", "left_shoulder_abductor", 
                           "left_shoulder_rotator", "left_elbow",
                           "left_wrist_rotator", "left_wrist_pitch", "left_wrist_yaw", 
                           "right_shoulder_extensor", "right_shoulder_abductor", 
                           "right_shoulder_rotator", "right_elbow", "right_wrist_rotator",
                           "right_wrist_pitch", "right_wrist_yaw"]

            index = 0
            for jointError in self.postureError:
                errorRad = jointError
                errorDeg = errorRad / 3.14 * 180
                entry = [str(index), JOINT_NAMES[index], str(errorRad), str(errorDeg)]
                resultList.append(entry)
                index = index + 1
        
            col_width = max(len(word) for row in resultList for word in row) + 2  # padding
            resultTable =""
            for row in resultList:
                resultTable = resultTable + "\n    " + "".join(word.ljust(col_width) for word in row) 
    
            index = raw_input("Current errors are:\n"\
                              " - Posture task:    {0}\n"\
                              " - Right hand Cartesian position:\n    {1}\n"\
                              " - Left hand Cartesian position:\n    {2}\n"\
                              " - Right hand orientation:\n    {3}\n"\
                              " - Left hand orientation:\n    {4}\n"\
                              "If error is low undo e-stop then type enter, otherwise type 'r' to retry or 'q' to quit.\n".format(
                resultTable, self.rightCartesianPosError, self.leftCartesianPosError, 
                self.rightOrientationError, self.leftOrientationError))

            if "q" == index:
                print "Exiting..."
                return False
            elif "r" == index:
                self.issueTareCommands()
            else:
                done = True

        return True

    def goToReadyPosition(self):

        # Define the waypoints
        # Note waypoints are formatted as: [[x, y, z], ...]
        # rightHandCartesianWP = []
        # rightHandCartesianWP.append(self.currentRightCartesianPos)
        # rightHandCartesianWP.append([0.034156182237965314, -0.2536961667775097, 0.7942923566248334])
        # rightHandCartesianWP.append([-0.03852115301282585, -0.36702885542756375, 1.0044042662878492])
        # rightHandCartesianWP.append([-0.0275400056213187, -0.4346278435022028, 1.109258357008881])
        # rightHandCartesianWP.append([0.16786527968278075, -0.48763818929105546, 1.2849643133074693])
        # rightHandCartesianWP.append([0.2561600552895205, -0.36355117909588747, 1.2737345840311838])

        # rightHandOrientationWP = []
        # rightHandOrientationWP.append(self.currentRightOrientation)
        # rightHandOrientationWP.append([0.9239165069202464, -0.16720064850712463, 0.34412531348200354])
        # rightHandOrientationWP.append([0.7104721853318615, 0.5004153180136336, 0.4947866038678526])
        # rightHandOrientationWP.append([0.5901777988221774, 0.748119163684364, 0.3033280117391358])
        # rightHandOrientationWP.append([-0.043115032422085975, 0.9965622982370774, 0.07074376093816886])
        # rightHandOrientationWP.append([0.34390539739454545, -0.566640981750115, 0.7487636980010218])

        # leftHandCartesianWP = []
        # leftHandCartesianWP.append(self.currentLeftCartesianPos)
        # leftHandCartesianWP.append([0.06522162547135797, 0.2087452184746953, 0.8066377828255454])
        # leftHandCartesianWP.append([0.015620097078689351, 0.3747161583451485, 0.8287361488491304])
        # leftHandCartesianWP.append([0.0066811057812040595, 0.46860930804289314, 0.8709620294725516])
        # leftHandCartesianWP.append([0.0189928085640673, 0.5361597637277502, 0.9197612611264894])
        # leftHandCartesianWP.append([0.03933509850107448, 0.6522939157440671, 1.0567686481769587])
        # leftHandCartesianWP.append([0.13258074491519561, 0.6854466730645508, 1.2068411332390032])
        # leftHandCartesianWP.append([0.23203580696538584, 0.5995259011103453, 1.2716484739906841])
        # leftHandCartesianWP.append([0.26654957772855264, 0.4880079098197537, 1.3033126448189423])
        # leftHandCartesianWP.append([0.32798561222658845, 0.3259907480928649, 1.2942321804350336])

        # leftHandOrientationWP = []
        # leftHandOrientationWP.append(self.currentLeftOrientation)
        # leftHandOrientationWP.append([0.8698342557072543, -0.2710769041539282, 0.41219616644569773])
        # leftHandOrientationWP.append([0.9106065993605292, -0.3890214692723871, 0.13949164005848422])
        # leftHandOrientationWP.append([0.9295869859711186, -0.36310368089122885, 0.06343305476147368])
        # leftHandOrientationWP.append([0.9177973156876307, -0.39449964250649994, 0.04492348360071424])
        # leftHandOrientationWP.append([0.9458938489405184, -0.30743636405669694, 0.10376757004040324])
        # leftHandOrientationWP.append([0.7819903734474247, -0.47561138322193586, 0.4028459606168016])
        # leftHandOrientationWP.append([0.19446889146452803, -0.5476779230575118, 0.813775609641852])
        # leftHandOrientationWP.append([-0.09223469751157737, -0.4569635179606584, 0.8846903999863268])
        # leftHandOrientationWP.append([-0.4312431770418993, -0.6624029322988932, 0.6125778950767296])

        jPosWP = []
        jPosWP.append(self.currentPosture)
        jPosWP.append([-0.01794845476545489,  -0.01794845476545489,  -0.16022875682719276, -0.028366232678305438, 0.050244446934761954, 0.5154665157243641,  -0.33255119610617534, 0.05010440837726589,   -0.018253115541772363, -0.16022875682719276, -0.028366232678305438, 0.050244446934761954, 0.5154665157243641,  -0.33255119610617534, 0.05010440837726589,   -0.018253115541772363])
        jPosWP.append([-0.01835755481596669,  -0.01835755481596669,  -0.2079656207335766,  0.29357242950906004,   0.04774540078163857,  0.4028570467687516,  -0.33545312171439673, 0.02454667484846745,   -0.025739450147506372, -0.2079656207335766,  0.29357242950906004,   0.04774540078163857,  0.4028570467687516,  -0.33545312171439673, 0.02454667484846745,   -0.025739450147506372])
        jPosWP.append([-0.018392780683976415, -0.018392780683976415, -0.21536831102820064, 0.4858139216602395,    0.04752910227054589,  0.3538691456458291,  -0.2606189020877619,  0.03851699336735694,   -0.009304268906057204, -0.21536831102820064, 0.4858139216602395,    0.04752910227054589,  0.3538691456458291,  -0.2606189020877619,  0.03851699336735694,   -0.009304268906057204])
        jPosWP.append([-0.018545167893084524, -0.018545167893084524, -0.20710870565527095, 0.6401132970059965,    0.0520458328895553,   0.35399683955047895, -0.2466998343348537,  0.06426357174636058,   0.032668663438796056,  -0.20710870565527095, 0.6401132970059965,    0.0520458328895553,   0.35399683955047895, -0.2466998343348537,  0.06426357174636058,   0.032668663438796056]) 
        jPosWP.append([-0.018356469848882423, -0.018356469848882423, -0.2162452342922192,  0.977742795946938,     0.051332200860391815, 0.3516444996185793,  0.005994626786898999, 0.06687364387429934,   0.2206448056367654,    -0.2162452342922192,  0.977742795946938,     0.051332200860391815, 0.3516444996185793,  0.005994626786898999, 0.06687364387429934,   0.2206448056367654])
        jPosWP.append([-0.01853752101703267,  -0.01853752101703267,  -0.08381303186931051, 1.2382056770859806,    0.07604977645605294,  0.5513916485119579,  0.2032285609484162,   0.0966689738445368,    0.19508567001135413,   -0.08381303186931051, 1.2382056770859806,    0.07604977645605294,  0.5513916485119579,  0.2032285609484162,   0.0966689738445368,    0.19508567001135413])  
        jPosWP.append([-0.018797824347083616, -0.018797824347083616, -0.05526364818674631, 1.2740213042786388,    0.1112481265579799,   1.0536733858716545,  0.6500190512067868,   0.1622132817288518,    0.47899325282370675,   -0.05526364818674631, 1.2740213042786388,    0.1112481265579799,   1.0536733858716545,  0.6500190512067868,   0.1622132817288518,    0.47899325282370675]) 
        jPosWP.append([-0.01844421650219398,  -0.01844421650219398,  -0.04650484640324995, 1.1951713147188352,    0.16733022331087605,  1.5113547364058941,  0.7241725349346869,   0.03489648599249408,   0.4296745935913513,    -0.04650484640324995, 1.1951713147188352,    0.16733022331087605,  1.5113547364058941,  0.7241725349346869,   0.03489648599249408,   0.4296745935913513]) 
        jPosWP.append([-0.017720064116942654, -0.017720064116942654, 0.471331735425394,    0.7301226149420205,    -0.2275738756636333,  1.683961941736368,  -0.014898011141877917, -0.034635678354951925, 0.5251207580826792,    0.471331735425394,    0.7301226149420205,    -0.2275738756636333,  1.683961941736368,  -0.014898011141877917, -0.034635678354951925, 0.5251207580826792])  

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

        index = raw_input("Go start position? Y/n\n")
        if index == "N" or index == "n":
            return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            # goalRightHandCartPos = None
            # goalRightHandOrientation = None
            goalJPos = None

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

        print "Done going to start position!"
        return True

    def grabMetalObject(self):
        """
        Executes the trajectory that moves the right arm to be around the metal object.
        """

        index = raw_input("Enable Cartesian position and orientation tasks? Y/n\n")
        if index == "N" or index == "n":
            return False  # quit

        # Enable the Cartesian position and orientation tasks
        self.rightCartesianTaskEnablePublisher.publish(self.enableMsg)
        self.leftCartesianTaskEnablePublisher.publish(self.enableMsg)
        self.rightOrientationTaskEnablePublisher.publish(self.enableMsg)
        self.leftOrientationTaskEnablePublisher.publish(self.enableMsg)


        # Wait for the current Cartesian position and orientation measurements to arrive
        pauseCount = 0
        printWarning = False
        while not rospy.is_shutdown() and (self.currentRightCartesianPos == None or \
            self.currentRightOrientation == None):
            time.sleep(0.5)
            pauseCount = pauseCount + 1
            if pauseCount > 5 and not printWarning:
                print "Waiting for current Cartesian position and orientation information..."
                printWarning = True

        # Create the WayPoints
        rightHandCartesianWP = []
        rightHandCartesianWP.append(self.currentRightCartesianPos)
        rightHandCartesianWP.append([0.3371965661920512, -0.33373481333031946, 1.271125553140791])
        rightHandCartesianWP.append([0.345363813886538, -0.2739238649764148, 1.1818351634211801])
        rightHandCartesianWP.append([0.357877414527656, -0.19737573441185602, 1.0326008134961595])
        rightHandCartesianWP.append([0.33951535715598313, -0.12704204972886532, 0.9798785163485518])

        rightHandOrientationWP = []
        rightHandOrientationWP.append(self.currentRightOrientation)
        rightHandOrientationWP.append([0.1837842485451516, -0.8509623841363126, 0.4920227339988841])
        rightHandOrientationWP.append([0.3620755018194659, -0.8936803420864066, 0.265022220107117])
        rightHandOrientationWP.append([0.3661516729783743, -0.9282533016701555, -0.0654122336691895])
        rightHandOrientationWP.append([0.38148946344697066, -0.9193971526142214, -0.09578447183027328])

        # leftHandCartesianWP = []
        # leftHandCartesianWP.append(self.currentLeftCartesianPos)
        # leftHandCartesianWP.append([0.12134979473280522, 0.19061502601446173, 0.8151659348041115])
        # leftHandCartesianWP.append([0.12134428914651882, 0.1905501244730116, 0.8150715027143034])
        # leftHandCartesianWP.append([0.11869376544232381, 0.19209110501596968, 0.8122738024925454])
        # leftHandCartesianWP.append([0.11863633128320361, 0.1921102161424108, 0.8123647135584903])

        # leftHandOrientationWP = []
        # leftHandOrientationWP.append(self.currentLeftOrientation)
        # leftHandOrientationWP.append([0.7828717574412116, -0.5820390901168132, 0.21986884494375528])
        # leftHandOrientationWP.append([0.7832259766760741, -0.5818424731427554, 0.21912646099212252])
        # leftHandOrientationWP.append([0.7921354933741186, -0.5831033464573026, 0.18031041979662832])
        # leftHandOrientationWP.append([0.791827258855825, -0.5834259433022426, 0.18062048835821184])

        # Create the trajectory generators
        rightHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandCartesianWP)
        rightHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(rightHandOrientationWP)
        # leftHandCartesianTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandCartesianWP)
        # leftHandOrientationTG = TrajectoryGeneratorCubicSpline.TrajectoryGeneratorCubicSpline(leftHandOrientationWP)

        TOTAL_TRAVEL_TIME = 10.0 # seconds
        rightHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        rightHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandCartesianTG.generateTrajectory(TOTAL_TRAVEL_TIME)
        # leftHandOrientationTG.generateTrajectory(TOTAL_TRAVEL_TIME)

        index = raw_input("Position hand around metal object? Y/n\n")
        if index == "N" or index == "n":
            return False  # quit

        # Follow the trajectories
        startTime = self.getTimeSeconds()
        done = False

        while not done and not rospy.is_shutdown():
            deltaTime = self.getTimeSeconds() - startTime

            goalRightHandCartPos = None
            goalRightHandOrientation = None
            # goalJPos = None

            if deltaTime >= TOTAL_TRAVEL_TIME:
                goalRightHandCartPos = rightHandCartesianTG.getLastPoint()
                goalRightHandOrientation = rightHandOrientationTG.getLastPoint()
                # goalLeftHandCartPos = leftHandCartesianTG.getLastPoint()
                # goalLeftHandOrientation = leftHandOrientationTG.getLastPoint()
                # goalJPos = jPosTG.getLastPoint()
                done = True
            else:
                goalRightHandCartPos = rightHandCartesianTG.getPoint(deltaTime)
                goalRightHandOrientation = rightHandOrientationTG.getPoint(deltaTime)
                # goalLeftHandCartPos = leftHandCartesianTG.getPoint(deltaTime)
                # goalLeftHandOrientation = leftHandOrientationTG.getPoint(deltaTime)
                # goalJPos = jPosTG.getPoint(deltaTime)

            # Save the new goals in ROS messages
            self.rightHandCartesianGoalMsg.data = goalRightHandCartPos
            self.rightHandOrientationGoalMsg.data = goalRightHandOrientation
            # self.leftHandCartesianGoalMsg.data = goalLeftHandCartPos
            # self.leftHandOrientationGoalMsg.data = goalLeftHandOrientation
            # self.postureGoalMsg.data = goalJPos

            # Publish the ROS messages
            self.rightCartesianTaskGoalPublisher.publish(self.rightHandCartesianGoalMsg)
            self.rightOrientationTaskGoalPublisher.publish(self.rightHandOrientationGoalMsg)
            # self.leftCartesianTaskGoalPublisher.publish(self.leftHandCartesianGoalMsg)
            # self.leftOrientationTaskGoalPublisher.publish(self.leftHandOrientationGoalMsg)
            self.postureTaskGoalPublisher.publish(self.postureGoalMsg)

            if not done:
                rospy.sleep(0.01) # 100Hz

        index = raw_input("Perform right hand power grasp? Y/n\n")
        if index == "N" or index == "n":
            return False  # quit

        self.rightHandCmdMsg.data = True
        self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)


        print "Done grabbing metal object!"
        return True

    def run(self):
        """
        Runs the demo 1 behavior.
        """

        if not self.doTare():
            return

        if not self.goToReadyPosition():
            return

        if not self.grabMetalObject():
            return

        print "Demo 1 done!"

        index = raw_input("Release power grasp? Y/n\n")
        if index == "N" or index == "n":
            return

        self.rightHandCmdMsg.data = False  # relax grasp
        self.rightHandCmdPublisher.publish(self.rightHandCmdMsg)

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo1_ProductDisassembly', anonymous=True)

    demo = Demo1_ProductDisassembly()
    # t = threading.Thread(target=demo.run)
    # t.start()
    demo.run()

    rospy.spin()  # just to prevent this node from exiting