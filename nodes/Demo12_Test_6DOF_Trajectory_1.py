#!/usr/bin/env python

'''
Tests 6-DOF end effector control of Dreamer.
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

# class EnablePowerGraspState(smach.State):
#     def __init__(self, dreamerInterface):
#         smach.State.__init__(self, outcomes=["done", "exit"])
#         self.dreamerInterface = dreamerInterface

#     def execute(self, userdata):
#         rospy.loginfo('Executing EnablePowerGraspState')
        
#         self.dreamerInterface.rightHandCmdMsg.data = True
#         self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
#         rospy.sleep(5) # allow fingers to move

#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return "done"

# class DisablePowerGraspState(smach.State):
#     def __init__(self, dreamerInterface):
#         smach.State.__init__(self, outcomes=["done", "exit"])
#         self.dreamerInterface = dreamerInterface

#     def execute(self, userdata):
#         rospy.loginfo('Executing DisablePowerGraspState')
        
#         self.dreamerInterface.rightHandCmdMsg.data = False
#         self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
#         rospy.sleep(5) # allow fingers to move

#         if rospy.is_shutdown():
#             return "exit"
#         else:
#             return "done"

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

class Demo12_Test6DOFTraj:
    def __init__(self, moveLeftWrist):
        self.dreamerInterface = DreamerInterface.DreamerInterface(enableUserPrompts = ENABLE_USER_PROMPTS, useQuaternionControl = True)
        self.moveLeftWrist = moveLeftWrist

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

        
        if self.moveLeftWrist:
            # trajGoToReady.addLHCartWP([0.08622489544461763, 0.22360231222226795, 0.7934163621381682])
            trajGoToReady.addLHCartWP([0.010699695355020408, 0.22536066137298344, 0.8562861550566412])
            trajGoToReady.addLHCartWP([-0.026638023361135896, 0.33383407815017047, 0.9135221201383533])
            trajGoToReady.addLHCartWP([-0.08617849265486421, 0.4402451225437884, 0.9754675861410689])
            trajGoToReady.addLHCartWP([-0.05204304229110306, 0.5996416860345987, 1.0982012681214877])
            trajGoToReady.addLHCartWP([0.061007345930971915, 0.5884724580787001, 1.1682974194347684])
            trajGoToReady.addLHCartWP([0.15108079170232028, 0.46643519282574375, 1.0502886727107463])
            trajGoToReady.addLHCartWP([0.23206373674683173, 0.3041702169446519, 1.036810558348982])

            # trajGoToReady.addLHOrientWP([0.01802213449386647, 0.9967872933718492, -0.0042739677342114295, 0.07792321631269339])
            trajGoToReady.addLHOrientWP([0.010453677275406942, 0.9848693274687852, -0.01976073854884583, 0.17185063763224337])
            trajGoToReady.addLHOrientWP([0.022286083719183395, 0.9342564895693433, 0.3000912246613683, 0.19134628047409033])
            trajGoToReady.addLHOrientWP([-0.09722638279234604, 0.880175671800969, 0.41746184674263176, 0.20387109597257896])
            trajGoToReady.addLHOrientWP([-0.44908780934152204, 0.6590655485037743, 0.2951189854398629, 0.5261725256122729])
            trajGoToReady.addLHOrientWP([-0.2339070742297226, 0.646228140161961, 0.33719371757346567, 0.643410497518567])
            trajGoToReady.addLHOrientWP([0.15984283253976206, 0.406495113351687, 0.5171487430690295, 0.7360497056918016])
            trajGoToReady.addLHOrientWP([0.37355107245515273, 0.5060237125709893, 0.5354549742064626, 0.5636377996382379])

            # trajGoToReady.addPostureWP(list(itertools.chain(
            #     [-0.03275314504282583, -0.03275314504282583], # torso
            #     [0.03274317117440041, 0.011602231279817963, 0.009987137501741144, 0.31907888267591866, -0.012925454078889415, -0.097173144842138, -0.0513850231577672],
            #     [-0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.032668738760182066, -0.032668738760182066], # torso
                [-0.4713492268239248, 0.014148106584920078, 0.012492955728161623, 1.04445407815899, -0.031945762634528606, -0.1271859844851621, -0.05000509982390801],
                [-0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.03291797737590012, -0.03291797737590012], # torso
                [-0.703576175318781, 0.23884000269666292, 0.12457296838073248, 1.2627799455657933, 0.7838487274657888, -0.15590835649130869, -0.03229828632986606],
                [-1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.033166757392176754, -0.033166757392176754], # torso
                [-0.9087964133858801, 0.34397507754602263, 0.42064903842036344, 1.2661775666752841, 1.0285552491309624, -0.15781547755068645, -0.03478647110358535],
                [-1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.03330523512928211, -0.03330523512928211], # torso
                [-0.6806261531353474, 0.4765680845728376, 1.1217446936741895, 1.1401767076176244, 0.6390890059675813, 0.2659865761525645, -0.06934208375500833],
                [-1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.033705424911517234, -0.033705424911517234], # torso
                [-0.626664062234498, 0.5878516316775118, 1.0600662840309016, 1.4439946517494875, 1.0255513177628615, -0.03136611877695937, -0.079239174924877],
                [-0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.033652912808301884, -0.033652912808301884], # torso
                [-0.14698826455788436, 0.14526557818679894, 0.8422186568881453, 1.5304386291430432, 1.3955815362103265, -0.03697456770608673, -0.27374519008564324],
                [-0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])))
            trajGoToReady.addPostureWP(list(itertools.chain(
                [-0.033391175597322645, -0.033391175597322645], # torso
                [-0.160614543625394, 0.08639860875837561, 0.2392745568736623, 1.6731526865171342, 1.5427627023849153, 0.006433807594178305, -0.045597032334302054],
                [-0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])))

        else:
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
        # Define the GoToIdle trajectory
        trajGoToIdle = Trajectory.Trajectory("TrajGoToIdle", TRAJECTORY_GO_TO_IDLE_EXECUTION_TIME)

        # Set the previous trajectory to be TrajGoToReady
        trajGoToIdle.setPrevTraj(trajGoToReady)
        
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

        if self.moveLeftWrist:
            trajGoToIdle.addLHCartWP([0.23206373674683173, 0.3041702169446519, 1.036810558348982])
            trajGoToIdle.addLHCartWP([0.15108079170232028, 0.46643519282574375, 1.0502886727107463])
            trajGoToIdle.addLHCartWP([0.061007345930971915, 0.5884724580787001, 1.1682974194347684])
            trajGoToIdle.addLHCartWP([-0.05204304229110306, 0.5996416860345987, 1.0982012681214877])
            trajGoToIdle.addLHCartWP([-0.08617849265486421, 0.4402451225437884, 0.9754675861410689])
            trajGoToIdle.addLHCartWP([-0.026638023361135896, 0.33383407815017047, 0.9135221201383533])
            trajGoToIdle.addLHCartWP([0.010699695355020408, 0.22536066137298344, 0.8562861550566412])
            # trajGoToIdle.addLHCartWP([0.08622489544461763, 0.22360231222226795, 0.7934163621381682])
            trajGoToIdle.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

            trajGoToIdle.addLHOrientWP([0.37355107245515273, 0.5060237125709893, 0.5354549742064626, 0.5636377996382379])
            trajGoToIdle.addLHOrientWP([0.15984283253976206, 0.406495113351687, 0.5171487430690295, 0.7360497056918016])
            trajGoToIdle.addLHOrientWP([-0.2339070742297226, 0.646228140161961, 0.33719371757346567, 0.643410497518567])
            trajGoToIdle.addLHOrientWP([-0.44908780934152204, 0.6590655485037743, 0.2951189854398629, 0.5261725256122729])
            trajGoToIdle.addLHOrientWP([-0.09722638279234604, 0.880175671800969, 0.41746184674263176, 0.20387109597257896])
            trajGoToIdle.addLHOrientWP([0.022286083719183395, 0.9342564895693433, 0.3000912246613683, 0.19134628047409033])
            trajGoToIdle.addLHOrientWP([0.010453677275406942, 0.9848693274687852, -0.01976073854884583, 0.17185063763224337])
            # trajGoToIdle.addLHOrientWP([0.01802213449386647, 0.9967872933718492, -0.0042739677342114295, 0.07792321631269339])
            trajGoToIdle.addLHOrientWP([0.0, 1.0, 0.0, 0.0])

            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.033391175597322645, -0.033391175597322645], 
                [-0.160614543625394, 0.08639860875837561, 0.2392745568736623, 1.6731526865171342, 1.5427627023849153, 0.006433807594178305, -0.045597032334302054],
                [-0.23757389535525622, 0.04591719434787172, 0.1278410467643636, 1.6277323745869778, 1.6772115700161672, -0.16424177713962002, -0.07271150445497984])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.033652912808301884, -0.033652912808301884], 
                [-0.14698826455788436, 0.14526557818679894, 0.8422186568881453, 1.5304386291430432, 1.3955815362103265, -0.03697456770608673, -0.27374519008564324],
                [-0.3473204748468209, 0.10776519554599014, 0.19223728101830065, 1.7387782499406619, 1.4955383045612167, -0.15524780645352, -0.06426266303552737])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.033705424911517234, -0.033705424911517234], 
                [-0.626664062234498, 0.5878516316775118, 1.0600662840309016, 1.4439946517494875, 1.0255513177628615, -0.03136611877695937, -0.079239174924877],
                [-0.48228016411548497, 0.21224044191487212, 0.669719319246093, 1.7273118869841055, 1.2982157441184288, -0.1522229498804123, -0.08341418135999394])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.03330523512928211, -0.03330523512928211], 
                [-0.6806261531353474, 0.4765680845728376, 1.1217446936741895, 1.1401767076176244, 0.6390890059675813, 0.2659865761525645, -0.06934208375500833],
                [-1.0632108817150043, 0.5069731198534251, 0.8282099405168639, 1.5441810293971316, 0.7963049502680233, 0.03203551846031133, 0.014448711039086787])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.033166757392176754, -0.033166757392176754], 
                [-0.9087964133858801, 0.34397507754602263, 0.42064903842036344, 1.2661775666752841, 1.0285552491309624, -0.15781547755068645, -0.03478647110358535],
                [-1.2063541697039177, 0.5105390055758586, 0.7413537899427806, 1.3121828016124317, -0.04628156004509675, 0.07479736649384078, -0.023032732690594346])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.03291797737590012, -0.03291797737590012], 
                [-0.703576175318781, 0.23884000269666292, 0.12457296838073248, 1.2627799455657933, 0.7838487274657888, -0.15590835649130869, -0.03229828632986606],
                [-1.027979301155709, 0.42664946889046196, 0.26838727684045416, 1.3431449354282685, -0.18598081367704852, 0.045863959573267304, 0.011991375307989063])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                [-0.032668738760182066, -0.032668738760182066], 
                [-0.4713492268239248, 0.014148106584920078, 0.012492955728161623, 1.04445407815899, -0.031945762634528606, -0.1271859844851621, -0.05000509982390801],
                [-0.6433949416194826, 0.09891907709380923, 0.02617480781444035, 1.042555339864615, -0.09837493173922425, 0.042368789771995014, 0.020423394398665785])))
            # trajGoToIdle.addPostureWP(list(itertools.chain(
            #     [-0.03275314504282583, -0.03275314504282583], 
            #     [0.03274317117440041, 0.011602231279817963, 0.009987137501741144, 0.31907888267591866, -0.012925454078889415, -0.097173144842138, -0.0513850231577672],
            #     [-0.2703826362458247, -0.0065433902706428905, 0.041972401461917844, 0.6110144337952284, 0.019912862716942595, 0.044322813664107175, 0.02367545574623616])))
            trajGoToIdle.addPostureWP(list(itertools.chain(
                DEFAULT_POSTURE_TORSO, 
                DEFAULT_POSTURE_LEFT_ARM, 
                DEFAULT_POSTURE_RIGHT_ARM)))

        else:
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
        stateGoToIdle = TrajectoryState(self.dreamerInterface, trajGoToIdle)
        stateUserInput = UserInputState()
        stateExit = ExitState()

        # ==============================================================================================
        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReadyState", stateGoToReady,
                transitions={'done':'UserInputState',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdleState", stateGoToIdle,
                transitions={'done':'ExitState',
                             'exit':'exit'})
            smach.StateMachine.add("UserInputState", stateUserInput,
                transitions={'done':'GoToIdleState',
                             'exit':'exit'})
            smach.StateMachine.add("ExitState", stateExit,
                transitions={'done':'GoToReadyState',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 12 behavior.
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

        print "Demo 12 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":

    # Load the defaults
    moveLeftWrist = False

    usageStr = "python Demo12_Test_6DOF_Trajectory_1.py [parameters]\n"\
               "Valid parameters include:\n"\
               "  -h Prints this help message\n"\
               "  -l include left wrist\n"

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

    rospy.init_node('Demo12_Test_6DOF_Traj', anonymous=True)

    demo = Demo12_Test6DOFTraj(moveLeftWrist)
    demo.run()

    
