#!/usr/bin/env python

'''
Uses 6-DOF end effector control to grab a box.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
import smach
import smach_ros

from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension, Bool, Int32

import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import DreamerInterface
import Trajectory
import TrajectoryGeneratorCubicSpline

import itertools  # for merging multiple lists

ENABLE_USER_PROMPTS = False

# Shoulder abductors about 10 degrees away from body and elbows bent 90 degrees
# DEFAULT_POSTURE = [0.0, 0.0,                                    # torso
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0,  # left arm
#                    0.0, 0.174532925, 0.0, 1.57, 0.0, 0.0, 0.0]  # right arm

# Shoulder abductors and elbows at about 10 degrees
DEFAULT_POSTURE_TORSO     = [0.0, 0.0]
DEFAULT_POSTURE_LEFT_ARM  = [0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]
DEFAULT_POSTURE_RIGHT_ARM = [0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0]

# The time each trajectory should take
TRAJECTORY_GO_TO_READY_EXECUTION_TIME = 10.0
TRAJECTORY_GO_TO_IDLE_EXECUTION_TIME = 10.0
TRAJECTORY_GRAB_BOX_EXECUTION_TIME = 5.0
TRAJECTORY_LIFT_BOX_EXECUTION_TIME = 5.0

class TrajectoryState(smach.State):
    def __init__(self, dreamerInterface, traj):
        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface
        self.traj = traj

    def execute(self, userdata):
        rospy.loginfo('Executing TrajectoryState')

        if self.dreamerInterface.followTrajectory(self.traj):
            return "done"
        else:
            return "exit"

class UserInputState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        rospy.loginfo('Executing UserInputState')

        index = raw_input("Type any key to continue or 'q' to quit\n")
        if index == "q" or index == "Q":
            return "exit"
        else:
            return "done"

class ExitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["done", "exit"])

    def execute(self, userdata):
        rospy.loginfo('Executing ExitState')

        index = raw_input("Repeat? (Y/n)\n")
        if index == "N" or index == "n":
            return "exit"
        else:
            return "done"

class EnablePowerGraspState(smach.State):
    def __init__(self, dreamerInterface):
        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface

    def execute(self, userdata):
        rospy.loginfo('Executing EnablePowerGraspState')
        
        self.dreamerInterface.rightHandCmdMsg.data = True
        self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
        rospy.sleep(5) # allow fingers to move

        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class DisablePowerGraspState(smach.State):
    def __init__(self, dreamerInterface):
        smach.State.__init__(self, outcomes=["done", "exit"])
        self.dreamerInterface = dreamerInterface

    def execute(self, userdata):
        rospy.loginfo('Executing DisablePowerGraspState')
        
        self.dreamerInterface.rightHandCmdMsg.data = False
        self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
        rospy.sleep(5) # allow fingers to move

        if rospy.is_shutdown():
            return "exit"
        else:
            return "done"

class SleepState(smach.State):
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

class Demo13_GraspBox:
    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(enableUserPrompts = ENABLE_USER_PROMPTS, useQuaternionControl = True)

    def createFSM(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        trajGoToReady = Trajectory.Trajectory("TrajGoToReady", TRAJECTORY_GO_TO_READY_EXECUTION_TIME)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        trajGoToReady.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        trajGoToReady.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.setInitRHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.setInitLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.setInitPostureWP(list(itertools.chain(
            DEFAULT_POSTURE_TORSO, 
            DEFAULT_POSTURE_LEFT_ARM, 
            DEFAULT_POSTURE_RIGHT_ARM)))
        
        trajGoToReady.addRHCartWP([0.03770122239874958, -0.2198480697094594, 0.8117255494770094])
        trajGoToReady.addRHCartWP([-0.040412081820136, -0.26397896734315346, 0.8614132694842028])
        trajGoToReady.addRHCartWP([-0.08745294968918928, -0.4212025201185584, 0.9700075682372389])
        trajGoToReady.addRHCartWP([-0.11703885123141751, -0.5441284402599831, 1.074238114641072])
        trajGoToReady.addRHCartWP([-0.028571575782775867, -0.5331226739243937, 1.1077348538402065])
        trajGoToReady.addRHCartWP([0.10443412373794271, -0.43466008783175114, 1.0813821843583091])
        trajGoToReady.addRHCartWP([0.18110137021381892, -0.2940613557332989, 1.0518617028889898])
        trajGoToReady.addRHCartWP([0.2113219018061716, -0.2643487426168306, 1.0374345284989988])

        trajGoToReady.addRHOrientWP([0.015314958442464818, 0.9742020083686487, -0.03310730940157143, 0.2227101367306515])
        trajGoToReady.addRHOrientWP([0.06315100311214952, 0.963529935854198, 0.0732094285307895, 0.24952433366894244])
        trajGoToReady.addRHOrientWP([0.2237161097085336, 0.9227471407807557, 0.22962971144883815, 0.2139135621173723])
        trajGoToReady.addRHOrientWP([0.39813383612342007, 0.8861239319509271, 0.141108980196644, 0.19068844082781058])
        trajGoToReady.addRHOrientWP([0.28435745155745307, 0.7917325092960383, -0.22346213905397447, 0.49230594743205314])
        trajGoToReady.addRHOrientWP([-0.15757388336647615, 0.5786146703417355, -0.45588708135956824, 0.6576796359897327])
        trajGoToReady.addRHOrientWP([-0.428763505205464, 0.5658049764829423, -0.4585747035651799, 0.5345426329500469])
        trajGoToReady.addRHOrientWP([0.5154210756453367, -0.5011947096853305, 0.5117860403981165, -0.47034033063055375])

        # left arm does not move
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToReady.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.03275314504282583, -0.03275314504282583], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.032668738760182066, -0.032668738760182066], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.03291797737590012, -0.03291797737590012], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.033166757392176754, -0.033166757392176754], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.03330523512928211, -0.03330523512928211], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.033705424911517234, -0.033705424911517234], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.033652912808301884, -0.033652912808301884], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])))
        trajGoToReady.addPostureWP(list(itertools.chain(
            [-0.033391175597322645, -0.033391175597322645], # torso
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])))

        # ==============================================================================================
        # Define the GrabBox trajectory
        trajGrabBox = Trajectory.Trajectory("TrajGrabBox", TRAJECTORY_GRAB_BOX_EXECUTION_TIME)

        trajGrabBox.setPrevTraj(trajGoToReady)

        trajGrabBox.addRHCartWP([0.24942109052382577, -0.25178385883304694, 1.0349451234548024])
        trajGrabBox.addRHCartWP([0.28142671550566234, -0.16419131273488383, 1.0007061260862327])
        trajGrabBox.addRHCartWP([0.2819871323937845, -0.0797792605404702, 1.0106879915905906])
        trajGrabBox.addRHCartWP([0.29176799505944284, -0.07022751146154063, 0.9720351794923792])
        trajGrabBox.addRHCartWP([0.24561624, -0.08057755, 1.1245961])   # peggy's position

        trajGrabBox.addRHOrientWP([0.54610373780756, -0.5066712957913012, 0.4472727269061978, -0.49497678060659517])
        trajGrabBox.addRHOrientWP([-0.4589709231622269, 0.6335632260701844, -0.3313524729975807, 0.5273982071466883])
        trajGrabBox.addRHOrientWP([-0.2952853269090797, 0.6491257407861527, -0.20169226418880165, 0.6713885454246454])
        trajGrabBox.addRHOrientWP([-0.21496655789808106, 0.6613310028274648, -0.005120190020180569, 0.7186128772425581])
        trajGrabBox.addRHOrientWP([-0.12418, -0.81758, 0.15326, -0.54097])  # peggy's orientation

        # left arm does not move
        trajGrabBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGrabBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGrabBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGrabBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajGrabBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGrabBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGrabBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGrabBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

        trajGrabBox.addPostureWP(list(itertools.chain(
            [-0.02951249080600518, -0.02951249080600518],
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.10410898486497548, 0.039994795025061175, 0.08377091614106322, 1.5088915595007792, 1.5808478627366467, -0.1940352968361497, -0.19512235694815247])))
        trajGrabBox.addPostureWP(list(itertools.chain(
            [-0.029000748173572167, -0.029000748173572167],
            DEFAULT_POSTURE_LEFT_ARM,
            [0.05099269121268372, 0.020053946717330424, -0.22523576453372868, 1.2344932999744034, 1.2458418531982232, 0.055738767503506585, -0.20544301193957204])))
        trajGrabBox.addPostureWP(list(itertools.chain(
            [-0.02890664538611541, -0.02890664538611541],
            DEFAULT_POSTURE_LEFT_ARM,
            [0.16720498644145737, 0.013529434975838748, -0.5762216634377006, 1.1688467366099806, 0.849305224439494, 0.49444249215498154, 0.13067752351334055])))
        trajGrabBox.addPostureWP(list(itertools.chain(
            [-0.02869732778818223, -0.02869732778818223],
            DEFAULT_POSTURE_LEFT_ARM,
            [0.23852987034676032, -0.05326764808955923, -0.5767480869580505, 0.9118877075637716, 0.43271609525555427, 0.5497642556657503, 0.08140717201110272])))

        # ==============================================================================================
        # Define the LiftBox trajectory
        trajLiftBox = Trajectory.Trajectory("TrajLiftBox", TRAJECTORY_LIFT_BOX_EXECUTION_TIME)

        trajLiftBox.setPrevTraj(trajGrabBox)

        trajLiftBox.addRHCartWP([0.24561624, -0.08057755, 1.1245961])   # peggy's position
        trajLiftBox.addRHCartWP([0.29176799505944284, -0.07022751146154063, 0.9720351794923792])
        trajLiftBox.addRHCartWP([0.2819871323937845, -0.0797792605404702, 1.0106879915905906])
        trajLiftBox.addRHCartWP([0.28142671550566234, -0.16419131273488383, 1.0007061260862327])
        trajLiftBox.addRHCartWP([0.24942109052382577, -0.25178385883304694, 1.0349451234548024])
        
        trajLiftBox.addRHOrientWP([-0.12418, -0.81758, 0.15326, -0.54097])  # peggy's orientation
        trajLiftBox.addRHOrientWP([-0.21496655789808106, 0.6613310028274648, -0.005120190020180569, 0.7186128772425581])
        trajLiftBox.addRHOrientWP([-0.2952853269090797, 0.6491257407861527, -0.20169226418880165, 0.6713885454246454])
        trajLiftBox.addRHOrientWP([-0.4589709231622269, 0.6335632260701844, -0.3313524729975807, 0.5273982071466883])
        trajLiftBox.addRHOrientWP([0.54610373780756, -0.5066712957913012, 0.4472727269061978, -0.49497678060659517])

        # left arm does not move
        trajLiftBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajLiftBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajLiftBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajLiftBox.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajLiftBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajLiftBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajLiftBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajLiftBox.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

        trajLiftBox.addPostureWP(list(itertools.chain(
            [-0.02869732778818223, -0.02869732778818223], 
            DEFAULT_POSTURE_LEFT_ARM,
            [0.23852987034676032, -0.05326764808955923, -0.5767480869580505, 0.9118877075637716, 0.43271609525555427, 0.5497642556657503, 0.08140717201110272])))
        trajLiftBox.addPostureWP(list(itertools.chain(
            [-0.02890664538611541, -0.02890664538611541], 
            DEFAULT_POSTURE_LEFT_ARM,
            [0.16720498644145737, 0.013529434975838748, -0.5762216634377006, 1.1688467366099806, 0.849305224439494, 0.49444249215498154, 0.13067752351334055])))
        trajLiftBox.addPostureWP(list(itertools.chain(
            [-0.029000748173572167, -0.029000748173572167], 
            DEFAULT_POSTURE_LEFT_ARM,
            [0.05099269121268372, 0.020053946717330424, -0.22523576453372868, 1.2344932999744034, 1.2458418531982232, 0.055738767503506585, -0.20544301193957204])))
        trajLiftBox.addPostureWP(list(itertools.chain(
            [-0.02951249080600518, -0.02951249080600518], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.10410898486497548, 0.039994795025061175, 0.08377091614106322, 1.5088915595007792, 1.5808478627366467, -0.1940352968361497, -0.19512235694815247])))


        # ==============================================================================================
        # Define the GoToIdle trajectory
        trajGoToIdle = Trajectory.Trajectory("TrajGoToIdle", TRAJECTORY_GO_TO_IDLE_EXECUTION_TIME)

        # Set the previous trajectory to be TrajGoToReady
        trajGoToIdle.setPrevTraj(trajLiftBox)
        
        trajGoToIdle.addRHCartWP([0.2113219018061716, -0.2643487426168306, 1.0374345284989988])
        trajGoToIdle.addRHCartWP([0.18110137021381892, -0.2940613557332989, 1.0518617028889898])
        trajGoToIdle.addRHCartWP([0.10443412373794271, -0.43466008783175114, 1.0813821843583091])
        trajGoToIdle.addRHCartWP([-0.028571575782775867, -0.5331226739243937, 1.1077348538402065])
        trajGoToIdle.addRHCartWP([-0.11703885123141751, -0.5441284402599831, 1.074238114641072])
        trajGoToIdle.addRHCartWP([-0.08745294968918928, -0.4212025201185584, 0.9700075682372389])
        trajGoToIdle.addRHCartWP([-0.040412081820136, -0.26397896734315346, 0.8614132694842028])
        trajGoToIdle.addRHCartWP([0.03770122239874958, -0.2198480697094594, 0.8117255494770094])
        trajGoToIdle.addRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])

        trajGoToIdle.addRHOrientWP([0.5154210756453367, -0.5011947096853305, 0.5117860403981165, -0.47034033063055375])
        trajGoToIdle.addRHOrientWP([-0.428763505205464, 0.5658049764829423, -0.4585747035651799, 0.5345426329500469])
        trajGoToIdle.addRHOrientWP([-0.15757388336647615, 0.5786146703417355, -0.45588708135956824, 0.6576796359897327])
        trajGoToIdle.addRHOrientWP([0.28435745155745307, 0.7917325092960383, -0.22346213905397447, 0.49230594743205314])
        trajGoToIdle.addRHOrientWP([0.39813383612342007, 0.8861239319509271, 0.141108980196644, 0.19068844082781058])
        trajGoToIdle.addRHOrientWP([0.2237161097085336, 0.9227471407807557, 0.22962971144883815, 0.2139135621173723])
        trajGoToIdle.addRHOrientWP([0.06315100311214952, 0.963529935854198, 0.0732094285307895, 0.24952433366894244])
        trajGoToIdle.addRHOrientWP([0.015314958442464818, 0.9742020083686487, -0.03310730940157143, 0.2227101367306515])
        trajGoToIdle.addRHOrientWP([0.0, 1.0, 0.0, 0.0])

        # left arm does not move
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.033391175597322645, -0.033391175597322645], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.033652912808301884, -0.033652912808301884], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.033705424911517234, -0.033705424911517234], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.03330523512928211, -0.03330523512928211], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.033166757392176754, -0.033166757392176754], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.03291797737590012, -0.03291797737590012], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.032668738760182066, -0.032668738760182066], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            [-0.03275314504282583, -0.03275314504282583], 
            DEFAULT_POSTURE_LEFT_ARM,
            [-0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])))
        trajGoToIdle.addPostureWP(list(itertools.chain(
            DEFAULT_POSTURE_TORSO, 
            DEFAULT_POSTURE_LEFT_ARM, 
            DEFAULT_POSTURE_RIGHT_ARM)))

        # ==============================================================================================
        # define the states
        stateGoToReady = TrajectoryState(self.dreamerInterface, trajGoToReady)
        stateGrabBox = TrajectoryState(self.dreamerInterface, trajGrabBox)
        stateLiftBox = TrajectoryState(self.dreamerInterface, trajLiftBox)
        stateGoToIdle = TrajectoryState(self.dreamerInterface, trajGoToIdle)
        stateUserInput1 = UserInputState()
        stateUserInput2 = UserInputState()
        stateUserInput3 = UserInputState()
        stateUserInput4 = UserInputState()
        stateUserInput5 = UserInputState()
        stateCloseHand = EnablePowerGraspState(self.dreamerInterface)
        stateOpenHand = DisablePowerGraspState(self.dreamerInterface)
        stateExit = ExitState()

        # ==============================================================================================
        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReadyState", stateGoToReady,
                transitions={'done':'UserInputState1',
                             'exit':'exit'})

            smach.StateMachine.add("UserInputState1", stateUserInput1,
                transitions={'done':'GrabBoxState',
                             'exit':'exit'})

            smach.StateMachine.add("GrabBoxState", stateGrabBox,
                transitions={'done':'UserInputState2',
                             'exit':'exit'})

            smach.StateMachine.add("UserInputState2", stateUserInput2,
                transitions={'done':'CloseHandState',
                             'exit':'exit'})

            smach.StateMachine.add("CloseHandState", stateCloseHand,
                transitions={'done':'UserInputState3',
                             'exit':'exit'})            

            smach.StateMachine.add("UserInputState3", stateUserInput3,
                transitions={'done':'LiftBoxState',
                             'exit':'exit'})

            smach.StateMachine.add("LiftBoxState", stateLiftBox,
                transitions={'done':'UserInputState4',
                             'exit':'exit'})

            smach.StateMachine.add("UserInputState4", stateUserInput4,
                transitions={'done':'OpenHandState',
                             'exit':'exit'})

            smach.StateMachine.add("OpenHandState", stateOpenHand,
                transitions={'done':'UserInputState5',
                             'exit':'exit'})

            smach.StateMachine.add("UserInputState5", stateUserInput5,
                transitions={'done':'GoToIdleState',
                             'exit':'exit'})

            smach.StateMachine.add("GoToIdleState", stateGoToIdle,
                transitions={'done':'ExitState',
                             'exit':'exit'})
            
            smach.StateMachine.add("ExitState", stateExit,
                transitions={'done':'GoToReadyState',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 13 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(list(itertools.chain(
            DEFAULT_POSTURE_TORSO, 
            DEFAULT_POSTURE_LEFT_ARM, 
            DEFAULT_POSTURE_RIGHT_ARM))):
            return

        self.createFSM()

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', self.fsm, '/SM_ROOT')
        sis.start()

        index = raw_input("Start demo? Y/n\n")
        if index == "N" or index == "n":
            return

        outcome = self.fsm.execute()

        print "Demo 13 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":

    # Load the defaults
    moveLeftWrist = False

    usageStr = "python {0} [parameters]\n"\
               "Valid parameters include:\n"\
               "  -h Prints this help message\n".format(__file__)

    try:
        opts, args = getopt.getopt(sys.argv[1:],"hl")
    except getopt.GetoptError:
       print usageStr
       sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':
            print usageStr
            sys.exit()
        elif opt == '-l':
            moveLeftWrist = True
        else:
            print "main: Unknown parameter: {0}\nUsage: {1}".format(arg, usageStr)
            sys.exit(2)

    rospy.init_node('Demo13_GraspBox', anonymous=True)

    demo = Demo13_GraspBox()
    demo.run()

    
