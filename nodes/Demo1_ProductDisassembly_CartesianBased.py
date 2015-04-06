#!/usr/bin/env python

'''
This uses both posture and orientation control.
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
TIME_GRAB_TUBE = 3.0
TIME_GRAB_VALVE = 3.0
TIME_REMOVE_VALVE = 3.0
TIME_REMOVE_LEFT_HAND = 3.0
TIME_STORE_TUBE = 3.0
TIME_REMOVE_RIGHT_HAND = 3.0
TIME_GO_TO_IDLE = 7.0

class TrajectoryState(smach.State):
    def __init__(self, dreamerInterface, goodResult, traj):
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

class RightHandPowerGraspState(smach.State):
    def __init__(self, dreamerInterface, goodResult, doGrasp, includeIndexFinger, includeMiddleFinger):
        smach.State.__init__(self, outcomes=[goodResult, "exit"])
        self.dreamerInterface = dreamerInterface
        self.goodResult = goodResult
        self.doGrasp = doGrasp
        self.includeIndexFinger = includeIndexFinger
        self.includeMiddleFinger = includeMiddleFinger

    def execute(self, userdata):
        rospy.loginfo('Executing RightHandPowerGraspState')

        if self.doGrasp:
            doPowerGrasp = True
            if ENABLE_USER_PROMPTS:
                index = raw_input("Perform right hand power grasp? Y/n\n")
                if index == "N" or index == "n":
                    doPowerGrasp = False
            
            if doPowerGrasp:
                self.dreamerInterface.rightIndexFingerCmdMsg.data = self.includeIndexFinger
                self.dreamerInterface.selectIndexFingerPublisher.publish(self.dreamerInterface.rightIndexFingerCmdMsg)
    
                self.dreamerInterface.rightMiddleFingerCmdMsg.data = self.includeMiddleFinger
                self.dreamerInterface.selectMiddleFingerPublisher.publish(self.dreamerInterface.rightMiddleFingerCmdMsg)
    
                self.dreamerInterface.rightHandCmdMsg.data = True
                self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        else:
            self.dreamerInterface.rightHandCmdMsg.data = False
            self.dreamerInterface.rightHandCmdPublisher.publish(self.dreamerInterface.rightHandCmdMsg)
        
        if rospy.is_shutdown():
            return "exit"
        else:
            return self.goodResult

class LeftGripperGraspState(smach.State):
    def __init__(self, dreamerInterface, goodResult, doGrasp):
        smach.State.__init__(self, outcomes=[goodResult, "exit"])
        self.dreamerInterface = dreamerInterface
        self.goodResult = goodResult
        self.doGrasp = doGrasp

    def execute(self, userdata):
        rospy.loginfo("Executing LeftGripperGraspState")

        if self.doGrasp:
            performGrasp = True
            if ENABLE_USER_PROMPTS:
                index = raw_input("Perform left gripper power grasp? Y/n\n")
                if index == "N" or index == "n":
                    performGrasp = False
            
            if performGrasp:
                self.dreamerInterface.leftGripperCmdMsg.data = True
                self.dreamerInterface.leftGripperCmdPublisher.publish(self.dreamerInterface.leftGripperCmdMsg)
        else:
            releaseGripper = True
            if ENABLE_USER_PROMPTS:
                index = raw_input("Release left gripper power grasp? Y/n\n")
                if index == "N" or index == "n":
                    releaseGripper = False

            if releaseGripper:
                self.dreamerInterface.leftGripperCmdMsg.data = False
                self.dreamerInterface.leftGripperCmdPublisher.publish(self.dreamerInterface.leftGripperCmdMsg)

        if rospy.is_shutdown():
            return "exit"
        else:
            return self.goodResult

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

class Demo1_ProductDisassembly:
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
        self.trajGrabTube = Trajectory.Trajectory("GrabTube", TIME_GRAB_TUBE)
        self.trajGrabTube.setPrevTraj(self.trajGoToReady)

        # Left hand does not move
        self.trajGrabTube.makeLHCartStatic(self.trajGoToReady)
        self.trajGrabTube.makeLHOrientStatic(self.trajGoToReady)
        
        # Specify the waypoints
        self.trajGrabTube.addRHCartWP([0.3137686413286708, -0.26659865390759796, 1.0541700108404664])
        self.trajGrabTube.addRHCartWP([0.2926328098812538, -0.22212659727858264, 0.9685956358633105])
        self.trajGrabTube.addRHCartWP([0.28750632029946943, -0.17027266952524717, 0.9597899484960192])
        self.trajGrabTube.addRHCartWP([0.28664480323526653, -0.1614844904659368, 0.9597645035426976])

        self.trajGrabTube.addRHOrientWP([0.6030193813610835, -0.6721560435204502, 0.42962062201648427])
        self.trajGrabTube.addRHOrientWP([0.6847262101203426, -0.7283419844816242, 0.025845131371348223])
        self.trajGrabTube.addRHOrientWP([0.8206856971797751, -0.5665754546656667, -0.0739407912788299])
        self.trajGrabTube.addRHOrientWP([0.830926574184253, -0.5512666962638427, -0.07527322169782114])
       
        self.trajGrabTube.addPostureWP([0.09594703765058178, 0.09594703765058178, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09817730939874109, 0.1020579374634571, 0.0836978735049272, 1.6235470575907778, 1.054005683347489, -0.6934016966989962, -0.4214573788290379])
        self.trajGrabTube.addPostureWP([0.09578187031551673, 0.09578187031551673, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.07827516108651086, 0.06968225019914681, -0.0651398024593994, 1.3456921412703295, 1.3295641014614135, -0.6445024104856519, -0.4748814187628949])
        self.trajGrabTube.addPostureWP([0.09578234092970726, 0.09578234092970726, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09157211596703656, 0.041108271773515705, -0.22384970463739684, 1.3076704033792463, 1.353903257753508, -0.7185241326180924, -0.454150460888528])
        self.trajGrabTube.addPostureWP([0.09590536736161434, 0.09590536736161434, 
                       -0.08569654146540764, 0.07021124925432169, 0,                    1.7194162945362514, 1.51, -0.07, -0.18,  # left arm
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # ==============================================================================================        
        self.trajGrabValve = Trajectory.Trajectory("GrabValve", TIME_GRAB_VALVE)
        self.trajGrabValve.setPrevTraj(self.trajGrabTube)

        # Right hand does not move
        self.trajGrabValve.makeRHCartStatic(self.trajGrabTube)
        self.trajGrabValve.makeRHOrientStatic(self.trajGrabTube)

        self.trajGrabValve.addLHCartWP([0.2558247304604975, 0.2949244102085959, 1.1042523046819483])
        self.trajGrabValve.addLHCartWP([0.24986753716571164, 0.2597244717600235, 1.075447863944082])
        self.trajGrabValve.addLHCartWP([0.29308174503554474, 0.24554806065234056, 1.0515318016118187])
        self.trajGrabValve.addLHCartWP([0.27, 0.26, 1.055])
        
        self.trajGrabValve.addLHOrientWP([-0.06355664672818866, -0.3507624169373639, 0.9343052389454567])
        self.trajGrabValve.addLHOrientWP([-0.05743278073528967, -0.24859633215529697, 0.9669029627299452])
        self.trajGrabValve.addLHOrientWP([-0.11088559335760977, -0.1963599862969318, 0.9742418287915992])
        self.trajGrabValve.addLHOrientWP([-0.0688185464892449, -0.10737620829573602, 0.9918338356555186])

        self.trajGrabValve.addPostureWP([0.09590536736161434, 0.09590536736161434, 
            -0.15114015512345808, 0.6757263833050331, -0.1512750672164876, 1.9078665565670625, 0.20486719263640862, 0.09583174147398679, -0.5754068224606331, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        self.trajGrabValve.addPostureWP([0.09590536736161434, 0.09590536736161434,  
            -0.13966183280088498, 0.4723217587608189, -0.17997921407313958, 1.8670358353680174, 0.1170707192657332, 0.11627257977324582, -0.6470561008686319, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        self.trajGrabValve.addPostureWP([0.09590536736161434, 0.09590536736161434,  
            0.03507646845214241, 0.32010870159025234, -0.18088026181707248, 1.6732313049447447, 0.06210729345798295, 0.1827444704387443, -0.9045072348249086, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])
        self.trajGrabValve.addPostureWP([0.09590536736161434, 0.09590536736161434, 
            0.002794296425643595, 0.2835844757183357, -0.18807469031404708, 1.6100747424098305, 0.11839413877386204, 0.248941084731985, -0.9437127120955863, 
            0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377])

        # ==============================================================================================        
        self.trajRemoveValve = Trajectory.Trajectory("RemoveValve", TIME_REMOVE_VALVE)
        self.trajRemoveValve.setPrevTraj(self.trajGrabValve)

        # Right hand does not move
        self.trajRemoveValve.makeRHCartStatic(self.trajGrabValve)
        self.trajRemoveValve.makeRHOrientStatic(self.trajGrabValve)

        self.trajRemoveValve.addLHCartWP([0.27, 0.26, 1.13])
        self.trajRemoveValve.addLHCartWP([0.41886442974443455, 0.3094769056991296, 1.2627290241255107])
        self.trajRemoveValve.addLHCartWP([0.48277345185719983, 0.3657639673352834, 1.300190357003811])
        self.trajRemoveValve.addLHCartWP([0.5243759641170038, 0.4270759618427376, 1.2178245652500823])

        self.trajRemoveValve.addLHOrientWP([-0.24277288830065594, 0.013363425163328217, 0.969991104894298])
        self.trajRemoveValve.addLHOrientWP([-0.4817466305582503, 0.11456662528305327, 0.8687891990103382])
        self.trajRemoveValve.addLHOrientWP([0.07967527065403097, -0.45239039250041857, 0.8882537835661182])
        self.trajRemoveValve.addLHOrientWP([0.48603077389668553, -0.31209439694412844, 0.8163156094437184])

        self.trajRemoveValve.addPostureWP([0.10580113260054594, 0.10580113260054594, 
                       0.23979689653764183, 0.5366671561387937, -0.24028499984085533, 1.9690385406401882, 0.4610325861244941, -0.01797598419331518, -1.0919871103215044, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveValve.addPostureWP([0.105459767394544, 0.105459767394544, 
                       0.6906767499136286, 0.41385749135856686, -0.03285041562768612, 1.640803065788199, 0.52732519608718, -0.01748961748122057, -1.0923600305447438, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveValve.addPostureWP([0.10545244652292243, 0.10545244652292243, 
                       1.011134048992465, 0.37656249916293216, 0.06403932455903373, 1.2390040299773695, -0.2878908639092968, -0.4295398542143753, -0.7719594438307211, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveValve.addPostureWP([0.1059528459107981, 0.1059528459107981, 
                       1.1823048236038691, 0.4227435883156559, 0.03803891004078773, 0.6072615761362916, -0.32622157205509805, -0.39518622920297136, -0.7588162298258935, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        # ==============================================================================================        
        self.trajRemoveLeftHand = Trajectory.Trajectory("RemoveLeftHand", TIME_REMOVE_LEFT_HAND)
        self.trajRemoveLeftHand.setPrevTraj(self.trajRemoveValve)

        # Right hand does not move
        self.trajRemoveLeftHand.makeRHCartStatic(self.trajRemoveValve)
        self.trajRemoveLeftHand.makeRHOrientStatic(self.trajRemoveValve)

        self.trajRemoveLeftHand.addLHCartWP([0.5243759641170038, 0.4270759618427376, 1.26])
        self.trajRemoveLeftHand.addLHCartWP([0.5660859116491781, 0.30329916134206, 1.3])
        self.trajRemoveLeftHand.addLHCartWP([0.5331769484913014, 0.33551507563836863, 1.4390766724027195])
        self.trajRemoveLeftHand.addLHCartWP([0.3717325873708269, 0.33162348107300504, 1.2861188455308357])
        self.trajRemoveLeftHand.addLHCartWP([0.2543733897875106, 0.29467616558006343, 1.233988175199949])
        self.trajRemoveLeftHand.addLHCartWP([0.27493886862283073, 0.2888653839749931, 1.1563093155705326])

        self.trajRemoveLeftHand.addLHOrientWP([0.41901011506443336, 0.16729650677786373, 0.892436217493225])
        self.trajRemoveLeftHand.addLHOrientWP([0.24882086601246373, 0.09105473725282154, 0.9642599294073292])
        self.trajRemoveLeftHand.addLHOrientWP([0.2330872682336373, 0.02584978972683424, 0.9721121919606085])
        self.trajRemoveLeftHand.addLHOrientWP([0.3801523299067776, -0.11787496189445643, 0.9173819811969435])
        self.trajRemoveLeftHand.addLHOrientWP([0.40154404672474, 0.16101160886995855, 0.9015750885805506])

        self.trajRemoveLeftHand.addPostureWP([0.10619477867763813, 0.10619477867763813, 
                       1.2202454432859893, 0.05738140848539306, 0.3189522271170663, 0.7232510537980966, -0.1726463028864427, -0.6028618209841994, 0.24500111186631893, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveLeftHand.addPostureWP([0.10643665665219065, 0.10643665665219065, 
                       1.5285064325031241, 0.12098014955475167, 0.29442826231308566, 0.9381167644779234, -0.33959274258259425, -0.9185511712204342, 0.10626589612738678, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveLeftHand.addPostureWP([0.10626453436778832, 0.10626453436778832, 
                       0.661416424392025, 0.1485420097610339, 0.3287541049538422, 1.8674898940472466, -0.13103595620300554, -0.9935935620370674, -0.21511362204881762, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveLeftHand.addPostureWP([0.10583243639915946, 0.10583243639915946, 
                       0.11249207204916184, 0.10479901770041052, 0.3332522844950901, 2.3037089105630066, -0.20471605919890376, -0.9940708398968128, -0.3071054312567586, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm
        self.trajRemoveLeftHand.addPostureWP([0.1059981788687132, 0.1059981788687132,
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, 
                       0.09105753863890241, 0.023808037050859456, -0.23396990791158995, 1.3070320542599851, 1.336118787118036, -0.7220768168517259, -0.45385861652866377]) # right arm

        # ==============================================================================================        
        self.trajStoreTube = Trajectory.Trajectory("StoreTube", TIME_STORE_TUBE)
        self.trajStoreTube.setPrevTraj(self.trajRemoveLeftHand)

        # Left hand does not move
        self.trajStoreTube.makeLHCartStatic(self.trajRemoveLeftHand)
        self.trajStoreTube.makeLHOrientStatic(self.trajRemoveLeftHand)

        self.trajStoreTube.addRHCartWP([0.27745790782530205, -0.24799403868875466, 1.0856900685019806])
        self.trajStoreTube.addRHCartWP([0.22994174073996312, -0.33516799382172185, 1.157388306768947])
        self.trajStoreTube.addRHCartWP([0.3561326251338532, -0.33946713949697493, 1.2243517460872133])
        self.trajStoreTube.addRHCartWP([0.46390462610893946, -0.293787775740201, 1.2749991528785545])
        self.trajStoreTube.addRHCartWP([0.5379763192787382, -0.2969524135307482, 1.2642177926977944])

        self.trajStoreTube.addRHOrientWP([0.06443474030825493, 0.2076489934596236, 0.8182663381789003])
        self.trajStoreTube.addRHOrientWP([0.06429600040269182, 0.20627739883206447, 0.8182111373492508])
        self.trajStoreTube.addRHOrientWP([0.0642089487626879, 0.20619008182347112, 0.8181108838396054])
        self.trajStoreTube.addRHOrientWP([0.06461697040941353, 0.20621809164001692, 0.8180000562311803])
        self.trajStoreTube.addRHOrientWP([0.0649257474773032, 0.20623027660103616, 0.8180329664395322])

        self.trajStoreTube.addPostureWP([0.10432430565653919, 0.10432430565653919, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.034478264115051754, 0.050983853806045615, 0.08104904608448296, 1.8817247736502365, 0.9521803557070976, -0.6771010281517622, -0.4043295777519238]) # right arm
        self.trajStoreTube.addPostureWP([0.10412618474968699, 0.10412618474968699, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.02640397181637044, -0.03708730110774295, 0.5450790880102646, 2.141813410250169, 0.59083630516349, -0.7125783320482355, -0.672996092257354])       # right arm
        self.trajStoreTube.addPostureWP([0.10495304042092034, 0.10495304042092034, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.5235748843032342, 0.027144300951222916, 0.4489218538612669, 1.8577598398066606, 0.2786073716961595, -0.7205390588078509, -0.6691257351559342])     # right arm
        self.trajStoreTube.addPostureWP([0.10529825572993265, 0.10529825572993265, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.893718014139769, 0.022189852043952615, 0.25391928553092946, 1.4869481709613361, -0.04994059069942265, -0.624367239720437, -0.4343711477894259])    # right arm
        self.trajStoreTube.addPostureWP([0.10504165418630391, 0.10504165418630391, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       1.1246908322214502, 0.04338531905479525, 0.260906295518494, 1.0195136153943856, -0.20861493889703248, -0.4437284194541821, -0.4310428017453589])     # right arm

        # ==============================================================================================        
        self.trajRemoveRightHand = Trajectory.Trajectory("RemoveRightHand", TIME_REMOVE_RIGHT_HAND)
        self.trajRemoveRightHand.setPrevTraj(self.trajStoreTube)

        # Left hand does not move
        self.trajRemoveRightHand.makeLHCartStatic(self.trajStoreTube)
        self.trajRemoveRightHand.makeLHOrientStatic(self.trajStoreTube)

        self.trajRemoveRightHand.addRHCartWP([0.5267938178777718, -0.3379240595874126, 1.3287458373790948])
        self.trajRemoveRightHand.addRHCartWP([0.41419485122905375, -0.31574762274337476, 1.2753847374072789])
        self.trajRemoveRightHand.addRHCartWP([0.32017232822372593, -0.2705999134573017, 1.156529294894129])
        self.trajRemoveRightHand.addRHCartWP([0.24477144224239697, -0.31430061008380034, 1.08248780350051])

        self.trajRemoveRightHand.addRHOrientWP([0.07391321441730378, 0.2058378093754104, 0.8180072409756564])
        self.trajRemoveRightHand.addRHOrientWP([0.07388394407368064, 0.20575418359715958, 0.8179989870031715])
        self.trajRemoveRightHand.addRHOrientWP([0.07136041758260608, 0.20565020252936395, 0.8167564895722128])
        self.trajRemoveRightHand.addRHOrientWP([0.07200104200391998, 0.20097007187421068, 0.8166377080331908])

        self.trajRemoveRightHand.addPostureWP([0.10569569813223582, 0.10569569813223582, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       1.2163328893732905, 0.22569420623412806, 0.12048969643886341, 1.0417177826656587, -0.007464719474735143, -0.5137164939035933, -0.3957809598196772])  # right arm
        self.trajRemoveRightHand.addPostureWP([0.10574401313049778, 0.10574401313049778, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.7397335721725239, 0.14117852326765132, 0.23724265240496786, 1.6944240945727442, 0.6944317983570832, -0.6549455740031402, -0.40816321974940045])     # right arm
        self.trajRemoveRightHand.addPostureWP([0.10439315309874873, 0.10439315309874873, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       0.16597850687837903, 0.21017650186148398, 0.06531500696244147, 1.9265909647670418, 0.8620543726577191, -0.4644505843758746, -0.6566952560811945])    # right arm
        self.trajRemoveRightHand.addPostureWP([0.10425184299714459, 0.10425184299714459, 
                       0.055975443762288614, -0.005876388217591141, 0.30103619726669223, 2.0797926209567303, 0.10425306276346165, -0.8026267888701227, -0.7932278546960052, # left arm
                       -0.11847755416302001, 0.06422356595783688, 0.327169061878712, 1.8993781491277486, 1.3082667536728438, -0.9510221581678175, -0.45723182354673425])   # right arm

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
        grabTubeState = TrajectoryState(self.dreamerInterface, "grabTubeDone", self.trajGrabTube)
        rightHandGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneGrabbingTube", True, False, False)
        grabValveState = TrajectoryState(self.dreamerInterface, "grabValveDone", self.trajGrabValve)
        leftGripperGraspState = LeftGripperGraspState(self.dreamerInterface, "doneGrabbingValve", True)
        middleFingerGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneMiddleFingerGrasp", True, False, True)
        sleepState = SleepState("doneSleep", 3)  # three seconds to allow middle finger to grasp tube
        removeValveState = TrajectoryState(self.dreamerInterface, "removeValveDone", self.trajRemoveValve)
        releaseGripperState = LeftGripperGraspState(self.dreamerInterface, "doneReleasingValve", False)
        indexFingerGraspState = RightHandPowerGraspState(self.dreamerInterface, "doneIndexFingerGrasp", True, True, True)
        removeLeftGripperState = TrajectoryState(self.dreamerInterface, "removeLeftGripperDone", self.trajRemoveLeftHand)
        storeTubeState = TrajectoryState(self.dreamerInterface, "storeTubeDone", self.trajStoreTube)
        dropTubeState = RightHandPowerGraspState(self.dreamerInterface, "doneDroppingTube", False, False, False)
        removeRightHandState = TrajectoryState(self.dreamerInterface, "removeRightHandDone", self.trajRemoveRightHand)
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

    rospy.init_node('Demo1_ProductDisassembly_CartesianBased', anonymous=True)

    demo = Demo1_ProductDisassembly()
    # t = threading.Thread(target=demo.run)
    # t.start()
    demo.run()

    
