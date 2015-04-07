#!/usr/bin/env python

'''
Enables users to telemanipulate Dreamer's end effectors via CARL.
Uses WBOSC configured with end effector Cartesian position and orientation.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
import smach
import smach_ros

from enum import IntEnum

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import DreamerInterface
import Trajectory
import TrajectoryGeneratorCubicSpline

ENABLE_USER_PROMPTS = False

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
                   0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]  # right arm

# The time each trajectory should take
TIME_GO_TO_READY = 5.0
TIME_GO_TO_IDLE = 7.0

# Define the commands that can be received from the CARL user interface
class Color(IntEnum):
    CMD_NONE = 0
    CMD_MOVE_RIGHT_HAND_FORWARD = 1
    CMD_MOVE_RIGHT_HAND_BACKWARD = 2
    CMD_MOVE_RIGHT_HAND_UP = 3
    CMD_MOVE_RIGHT_HAND_DOWN = 4
    CMD_MOVE_RIGHT_HAND_LEFT = 5
    CMD_MOVE_RIGHT_HAND_RIGHT = 6
    CMD_MOVE_LEFT_HAND_FORWARD = 7
    CMD_MOVE_LEFT_HAND_BACKWARD = 8
    CMD_MOVE_LEFT_HAND_UP = 9
    CMD_MOVE_LEFT_HAND_DOWN = 10 
    CMD_MOVE_LEFT_HAND_LEFT = 11
    CMD_MOVE_LEFT_HAND_RIGHT = 12

class TrajectoryState(smach.State):
    """
    A SMACH state that makes the robot follow a trajectory.
    """

    def __init__(self, dreamerInterface, goodResult, traj):
        """
        The constructor.

        Keyword Parameters:
          - dreamerInterface: The object to which to provide the trajectory.
          - goodResult: The string to return after the trajectory is 
                        successfully followed. Note that "exit" is returned 
                        if there is an error in following the trajectory.
          - traj: The trajectory to follow.
        """

        smach.State.__init__(self, outcomes=[goodResult, "exit"])
        self.dreamerInterface = dreamerInterface
        self.goodResult = goodResult
        self.traj = traj

    def execute(self, userdata):
        rospy.loginfo('Executing TrajectoryState')

        if self.dreamerInterface.followTrajectory(self.traj):
            return self.goodResult
        else:
            return "exit"

class AwaitCommandStateState(smach.State):
    """
    A SMACH state that waits for a command to arrive. It subscribes to a 
    ROS topic over which commands are published and triggers a transition
    based on the received command.
    """

    def __init__(self):
        """
        The constructor.
        """

        # Initialize parent class
        smach.State.__init__(self, outcomes=["move_right_hand_forward",
                                             "move_right_hand_backward",
                                             "move_right_hand_up",
                                             "move_right_hand_down",
                                             "move_right_hand_left",
                                             "move_right_hand_right",
                                             "move_left_hand_forward",
                                             "move_left_hand_backward",
                                             "move_left_hand_up",
                                             "move_left_hand_down",
                                             "move_left_hand_left",
                                             "move_left_hand_right",
                                             "exit"])
        # Initialize local variables
        self.rcvdCmd = False
        self.sleepPeriod = 0.5  # in seconds
        self.cmdNumber = CMD_NONE

        # Register a ROS topic listener
        self.demoCmdSubscriber  = rospy.Subscriber("/demo9/cmd", Int32, self.demoCmdCallback)
        self.demoDonePublisher = rospy.Publisher("/demo9/done",  Int32, queue_size=1)

    def demoCmdCallback(self, msg):
        """
        The command ROS topic subscriber's callback method.
        """

        self.cmdNumber = msg.data
        self.rcvdCmd = True

    def execute(self, userdata):
        rospy.loginfo('Executing await command...')

        while not self.rcvdCmd:
            rospy.sleep(self.sleepPeriod)

        if rospy.is_shutdown():
            return "exit"
        else:
            if self.cmd

# class RightHandPowerGraspState(smach.State):
#     def __init__(self, dreamerInterface, goodResult, doGrasp, includeIndexFinger, includeMiddleFinger):
#         smach.State.__init__(self, outcomes=[goodResult, "exit"])
#         self.dreamerInterface = dreamerInterface
#         self.goodResult = goodResult
#         self.doGrasp = doGrasp
#         self.includeIndexFinger = includeIndexFinger
#         self.includeMiddleFinger = includeMiddleFinger

#     def execute(self, userdata):
#         rospy.loginfo('Executing RightHandPowerGraspState')

#         if self.doGrasp:
#             doPowerGrasp = True
#             if ENABLE_USER_PROMPTS:
#                 index = raw_input("Perform right hand power grasp? Y/n\n")
#                 if index == "N" or index == "n":
#                     doPowerGrasp = False
            
#             if doPowerGrasp:
#                 self.dreamerInterface.rightIndexFingerCmdMsg.data = self.includeIndexFinger
#                 self.dreamerInterface.selectIndexFingerPublisher.publish(self.dreamerInterface.rightIndexFingerCmdMsg)
    
#                 self.dreamerInterface.rightMiddleFingerCmdMsg.data = self.includeMiddleFinger
#                 self.dreamerInterface.selectMiddleFingerPublisher.publish(self.dreamerInterface.rightMiddleFingerCmdMsg)
    
#                 self.dreamerInterface.rightHandCmdMsg.data = True
#                 self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
#         else:
#             self.dreamerInterface.rightHandCmdMsg.data = False
#             self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return self.goodResult

# class LeftGripperGraspState(smach.State):
#     def __init__(self, dreamerInterface, goodResult, doGrasp):
#         smach.State.__init__(self, outcomes=[goodResult, "exit"])
#         self.dreamerInterface = dreamerInterface
#         self.goodResult = goodResult
#         self.doGrasp = doGrasp

#     def execute(self, userdata):
#         rospy.loginfo("Executing LeftGripperGraspState")

#         if self.doGrasp:
#             performGrasp = True
#             if ENABLE_USER_PROMPTS:
#                 index = raw_input("Perform left gripper power grasp? Y/n\n")
#                 if index == "N" or index == "n":
#                     performGrasp = False
            
#             if performGrasp:
#                 self.dreamerInterface.leftGripperCmdMsg.data = True
#                 self.dreamerInterface.leftGripperCmdPublisher.publish(self.dreamerInterface.leftGripperCmdMsg)
#         else:
#             releaseGripper = True
#             if ENABLE_USER_PROMPTS:
#                 index = raw_input("Release left gripper power grasp? Y/n\n")
#                 if index == "N" or index == "n":
#                     releaseGripper = False

#             if releaseGripper:
#                 self.dreamerInterface.leftGripperCmdMsg.data = False
#                 self.dreamerInterface.leftGripperCmdPublisher.publish(self.dreamerInterface.leftGripperCmdMsg)

#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return self.goodResult

class SleepState(smach.State):
    """
    Makes the robot pause for a specified amount of time.
    """

    def __init__(self, goodResult, period):
        smach.State.__init__(self, outcomes=[goodResult, "exit"])
        self.goodResult = goodResult
        self.period = period

    def execute(self, userdata):
        rospy.loginfo("Executing SleepState")
        rospy.sleep(self.period)

        if rospy.is_shutdown():
            return "exit"
        else:
            return self.goodResult

class Demo9_CARL_Telemanipulation:
    """
    The primary class that implement's the demo's FSM.
    """

    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(ENABLE_USER_PROMPTS)

    def createTrajectories(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        self.trajGoToReady = Trajectory.Trajectory("GoToReady", TIME_GO_TO_READY)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        self.trajGoToReady.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        self.trajGoToReady.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.setInitRHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.setInitPostureWP(DEFAULT_POSTURE)
        
        self.trajGoToReady.addRHCartWP([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])
        self.trajGoToReady.addRHCartWP([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        self.trajGoToReady.addRHCartWP([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        self.trajGoToReady.addRHCartWP([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        self.trajGoToReady.addRHCartWP([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        self.trajGoToReady.addRHCartWP([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])

        self.trajGoToReady.addRHOrientWP([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        self.trajGoToReady.addRHOrientWP([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        self.trajGoToReady.addRHOrientWP([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        self.trajGoToReady.addRHOrientWP([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        self.trajGoToReady.addRHOrientWP([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        self.trajGoToReady.addRHOrientWP([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])

        self.trajGoToReady.addLHCartWP([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        self.trajGoToReady.addLHCartWP([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        self.trajGoToReady.addLHCartWP([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        self.trajGoToReady.addLHCartWP([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        self.trajGoToReady.addLHCartWP([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        self.trajGoToReady.addLHCartWP([0.25822435038901964, 0.25,               1.0461857180093073])

        self.trajGoToReady.addLHOrientWP([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])
        self.trajGoToReady.addLHOrientWP([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        self.trajGoToReady.addLHOrientWP([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        self.trajGoToReady.addLHOrientWP([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        self.trajGoToReady.addLHOrientWP([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        self.trajGoToReady.addLHOrientWP([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])

        self.trajGoToReady.addPostureWP([0.06826499288341317, 0.06826499288341317, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, 
                       -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])
        self.trajGoToReady.addPostureWP([0.0686363596318602,  0.0686363596318602,  
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, 
                       -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        self.trajGoToReady.addPostureWP([0.06804075180539401, 0.06804075180539401, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, 
                       -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        self.trajGoToReady.addPostureWP([0.06818415549992426, 0.06818415549992426, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, 
                       -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        self.trajGoToReady.addPostureWP([0.06794500584573498, 0.06794500584573498, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, 
                       -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        self.trajGoToReady.addPostureWP([0.06796522908004803, 0.06796522908004803,                                                                   # torso
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm

        # ==============================================================================================        
        self.trajGoToIdle = Trajectory.Trajectory("GoToIdle", TIME_GO_TO_IDLE)
        self.trajGoToIdle.setPrevTraj(self.trajRemoveRightHand)

        # 2015.01.06 Trajectory
        self.trajGoToIdle.addRHCartWP([0.25822435038901964, -0.1895604971725577, 1.0461857180093073])
        self.trajGoToIdle.addRHCartWP([0.21649227857092893, -0.3006839904787592, 1.1140502834793191])
        self.trajGoToIdle.addRHCartWP([0.11866831717348489, -0.4101100845056917, 1.209699047600146])
        self.trajGoToIdle.addRHCartWP([-0.03366873622218044, -0.40992725074781894, 1.1144948070701866])
        self.trajGoToIdle.addRHCartWP([-0.055152798770261954, -0.2907526623508046, 1.009663652974324])
        self.trajGoToIdle.addRHCartWP([0.019903910090688474, -0.28423307267223147, 0.9179288590591458])

        self.trajGoToIdle.addRHOrientWP([0.5409881394605172, -0.8191390472602035, 0.19063854336595773])
        self.trajGoToIdle.addRHOrientWP([0.260956993686226, -0.8736061290033836, 0.4107478287392042])
        self.trajGoToIdle.addRHOrientWP([0.19818667912613866, -0.8161433027447201, 0.5428002851895832])
        self.trajGoToIdle.addRHOrientWP([0.8994250702615956, 0.22626156457297464, 0.3739521993275524])
        self.trajGoToIdle.addRHOrientWP([0.8944226954968388, 0.33098423072776184, 0.3007615015086225])
        self.trajGoToIdle.addRHOrientWP([0.8950968852599132, 0.26432788250814326, 0.3590714922223199])
        
        self.trajGoToIdle.addLHCartWP([0.25822435038901964, 0.1895604971725577, 1.0461857180093073])
        self.trajGoToIdle.addLHCartWP([0.21649227857092893, 0.3006839904787592, 1.1140502834793191])
        self.trajGoToIdle.addLHCartWP([0.11866831717348489, 0.4101100845056917, 1.209699047600146])
        self.trajGoToIdle.addLHCartWP([-0.03366873622218044, 0.40992725074781894, 1.1144948070701866])
        self.trajGoToIdle.addLHCartWP([-0.055152798770261954, 0.2907526623508046, 1.009663652974324])
        self.trajGoToIdle.addLHCartWP([0.019903910090688474, 0.28423307267223147, 0.9179288590591458])
        
        self.trajGoToIdle.addLHOrientWP([0.5409881394605172, 0.8191390472602035, 0.19063854336595773])
        self.trajGoToIdle.addLHOrientWP([0.260956993686226, 0.8736061290033836, 0.4107478287392042])
        self.trajGoToIdle.addLHOrientWP([0.19818667912613866, 0.8161433027447201, 0.5428002851895832])
        self.trajGoToIdle.addLHOrientWP([0.8994250702615956, -0.22626156457297464, 0.3739521993275524])
        self.trajGoToIdle.addLHOrientWP([0.8944226954968388, -0.33098423072776184, 0.3007615015086225])
        self.trajGoToIdle.addLHOrientWP([0.8950968852599132, -0.26432788250814326, 0.3590714922223199])

        self.trajGoToIdle.addPostureWP([0.06796522908004803, 0.06796522908004803,                                                  # torso
                       -0.08569654146540764, 0.07021124925432169,                    0, 1.7194162945362514, 1.51, -0.07, -0.18,    # left arm
                       -0.08569654146540764, 0.07021124925432169, -0.15649686418494702, 1.7194162945362514, 1.51, -0.07, -0.18])   # right arm
        self.trajGoToIdle.addPostureWP([0.06794500584573498, 0.06794500584573498, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457, -0.24608246913199228, 0.13441397755549533, 0.2542869735593113,   2.0227000417984633, 1.3670468713459782,  -0.45978204939890815, 0.030219082955597457])
        self.trajGoToIdle.addPostureWP([0.06818415549992426, 0.06818415549992426, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628, -0.8497599545494692,  0.47079074342878563, 0.8355038507753617,   2.2318590905389852, 1.8475059506175733,  -0.405570582208143,   -0.0277359315904628])
        self.trajGoToIdle.addPostureWP([0.06804075180539401, 0.06804075180539401, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467, -1.3637873691001094,  0.3926057912988488,  0.575755053425441,    1.9732992187122156, 0.29999797251313004, -0.20309827518257023, 0.05586603055643467])
        self.trajGoToIdle.addPostureWP([0.0686363596318602,  0.0686363596318602,  -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179, -1.0914342991625676,  0.39040871074764566, -0.03720209764435387, 1.7583823306095314, 0.05438773164693069, -0.20257591921666193, 0.06386553930484179])
        self.trajGoToIdle.addPostureWP([0.06826499288341317, 0.06826499288341317, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836, -0.6249282444166423,  0.3079607416653748,  -0.1220981510225299,  1.3675006234559883, 0.06394316468492173, -0.20422693251592328, 0.06223224746326836])

    def createFSM(self):
        # define the states
        goToReadyState = TrajectoryState(self.dreamerInterface, "goToReadyDone", self.trajGoToReady)
        # grabTubeState = TrajectoryState(self.dreamerInterface, "grabTubeDone", self.trajGrabTube)
        # rightHandGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneGrabbingTube", True, False, False)
        # grabValveState = TrajectoryState(self.dreamerInterface, "grabValveDone", self.trajGrabValve)
        # leftGripperGraspState = LeftGripperGraspState(self.dreamerInterface, "doneGrabbingValve", True)
        # middleFingerGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneMiddleFingerGrasp", True, False, True)
        # sleepState = SleepState("doneSleep", 3)  # three seconds to allow middle finger to grasp tube
        # removeValveState = TrajectoryState(self.dreamerInterface, "removeValveDone", self.trajRemoveValve)
        # releaseGripperState = LeftGripperGraspState(self.dreamerInterface, "doneReleasingValve", False)
        # indexFingerGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneIndexFingerGrasp", True, True, True)
        # removeLeftGripperState = TrajectoryState(self.dreamerInterface, "removeLeftGripperDone", self.trajRemoveLeftHand)
        # storeTubeState = TrajectoryState(self.dreamerInterface, "storeTubeDone", self.trajStoreTube)
        # dropTubeState = RightHandPowerGraspState(self.dreamerInterface, "doneDroppingTube", False, False, False)
        # removeRightHandState = TrajectoryState(self.dreamerInterface, "removeRightHandDone", self.trajRemoveRightHand)
        goToIdleState = TrajectoryState(self.dreamerInterface, "doneGoToIdle", self.trajGoToIdle)

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReady", goToReadyState, 
                transitions={'goToReadyDone':'GrabTube',
                             'exit':'exit'})
            smach.StateMachine.add("GrabTube", grabTubeState, 
                transitions={'grabTubeDone':'RightHandInitialGraspState',
                             'exit':'exit'})
            smach.StateMachine.add("RightHandInitialGraspState", rightHandGraspState, 
                transitions={'doneGrabbingTube':'GrabValveState',
                             'exit':'exit'})
            smach.StateMachine.add("GrabValveState", grabValveState, 
                transitions={'grabValveDone':'CloseLeftGripperState',
                             'exit':'exit'})
            smach.StateMachine.add("CloseLeftGripperState", leftGripperGraspState, 
                transitions={'doneGrabbingValve':'AddMiddleFingerState',
                             'exit':'exit'})
            smach.StateMachine.add("AddMiddleFingerState", middleFingerGraspState, 
                transitions={'doneMiddleFingerGrasp':'SleepState',
                             'exit':'exit'})
            smach.StateMachine.add("SleepState", sleepState, 
                transitions={'doneSleep':'RemoveValveState',
                             'exit':'exit'})
            smach.StateMachine.add("RemoveValveState", removeValveState, 
                transitions={'removeValveDone':'ReleaseGripperState',
                             'exit':'exit'})
            smach.StateMachine.add("ReleaseGripperState", releaseGripperState, 
                transitions={'doneReleasingValve':'AddIndexFingerState',
                             'exit':'exit'})
            smach.StateMachine.add("AddIndexFingerState", indexFingerGraspState, 
                transitions={'doneIndexFingerGrasp':'RemoveLeftGripperState',
                             'exit':'exit'})
            smach.StateMachine.add("RemoveLeftGripperState", removeLeftGripperState, 
                transitions={'removeLeftGripperDone':'StoreTubeState',
                             'exit':'exit'})
            smach.StateMachine.add("StoreTubeState", storeTubeState, 
                transitions={'storeTubeDone':'DropTubeState',
                             'exit':'exit'})
            smach.StateMachine.add("DropTubeState", dropTubeState, 
                transitions={'doneDroppingTube':'RemoveRightHandState',
                             'exit':'exit'})
            smach.StateMachine.add("RemoveRightHandState", removeRightHandState, 
                transitions={'removeRightHandDone':'GoToIdleState',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdleState", goToIdleState, 
                transitions={'doneGoToIdle':'exit',
                             'exit':'exit'})

    def run(self):
        """
        Runs the Cartesian and orientation demo 1 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(DEFAULT_POSTURE):
            return

        self.createTrajectories()
        self.createFSM()

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', self.fsm, '/SM_ROOT')
        sis.start()

        index = raw_input("Start demo? Y/n\n")
        if index == "N" or index == "n":
            return

        outcome = self.fsm.execute()

        print "Cartesian based demo 1 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo9_CARL_Telemanipulation', anonymous=True)
    demo = Demo9_CARL_Telemanipulation()
    demo.run()