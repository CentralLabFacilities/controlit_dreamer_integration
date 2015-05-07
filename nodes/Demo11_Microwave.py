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
TIME_GRAB_CUP = 3.0
TIME_PUT_IN_MICROWAVE = 6.0
TIME_REMOVE_HAND = 5.0
TIME_GO_TO_IDLE = 7.0

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

class Demo11_Microwave:
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
        
        self.trajGoToReady.addRHCartWP([0.06151360655767004, -0.35473365103360105, 0.8502425772059228])
        self.trajGoToReady.addRHCartWP([0.06254085413347153, -0.4532215961822638, 0.8945440440105833])
        self.trajGoToReady.addRHCartWP([-0.008160835258889656, -0.6005836171844121, 0.9985345939863775])
        self.trajGoToReady.addRHCartWP([0.003423112199538006, -0.681337057256572, 1.1300195897888556])
        self.trajGoToReady.addRHCartWP([0.04839867727793537, -0.6603951419693301, 1.2135959637058509])
        self.trajGoToReady.addRHCartWP([0.10763113127238672, -0.5830871090733677, 1.2227260210222508])

        self.trajGoToReady.addRHOrientWP([0.94120364105694, 0.02827680106855363, 0.3366543161500953])
        self.trajGoToReady.addRHOrientWP([0.9297924855049255, -0.005304194800132501, 0.3680459202546538])
        self.trajGoToReady.addRHOrientWP([0.913268194970981, 0.06816699495862243, 0.4016148215053283])
        self.trajGoToReady.addRHOrientWP([0.7679241996028208, 0.07448883206392755, 0.6361948110147703])
        self.trajGoToReady.addRHOrientWP([-0.10814999445744948, -0.10571110403434188, 0.9884982251793345])
        self.trajGoToReady.addRHOrientWP([-0.24120758314246643, -0.7662394727458538, 0.5955635753139427])

        # left arm does not move
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajGoToReady.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajGoToReady.addLHOrientWP([1.0, 0.0, 0.0])

        self.trajGoToReady.addPostureWP([-0.03954143115888832, -0.03954143115888832, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.28346414329247743, 0.2048504698541909, 0.21860364450531333, 0.718082373248261, -0.11102142155659253, -0.172117336918965, -0.37211799355537634])
        self.trajGoToReady.addPostureWP([-0.04048733689039183, -0.04048733689039183, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.27984979680843436, 0.36103398957513305, 0.389633163825197, 0.7177444024590417, -0.13854734404518876, -0.17785365466147704, -0.3602301143413306])
        self.trajGoToReady.addPostureWP([-0.040489006645572125, -0.040489006645572125, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.4539624338879733, 0.71035327347394, 0.519467130845678, 0.59736417495704, 0.005555299350397968, 0.02166969972826118, -0.5727098351085058])
        self.trajGoToReady.addPostureWP([-0.04068427123165474, -0.04068427123165474, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.45058462509292335, 0.9028684991169388, 0.904537876784497, 0.6153717703520567, 0.011343155514405064, 0.025186976099833128, -0.5763605200378115])
        self.trajGoToReady.addPostureWP([-0.040635490486366214, -0.040635490486366214, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.4078734390604989, 0.8221066477245813, 1.1100485723106124, 1.0597316605132583, 0.7884301537079474, -0.14747641664135208, -0.730331478991633])
        self.trajGoToReady.addPostureWP([-0.04024681687220234, -0.04024681687220234, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.3123717152931635, 0.6522264537475179, 0.983735523618788, 1.4620676951129983, 1.5545854687714091, -0.6041193371140029, -0.25543158300352464])

        # ==============================================================================================
        self.trajGrabCup = Trajectory.Trajectory("GrabCup", TIME_GRAB_CUP)
        self.trajGrabCup.setPrevTraj(self.trajGoToReady)

        # Left hand does not move
        self.trajGrabCup.makeLHCartStatic(self.trajGoToReady)
        self.trajGrabCup.makeLHOrientStatic(self.trajGoToReady)
        
        # Specify the waypoints
        self.trajGrabCup.addRHCartWP([0.10407649596151439, -0.5645354310232097, 1.2184315253189777])
        self.trajGrabCup.addRHCartWP([0.10683714219993523, -0.5265491056895999, 1.151079340467994])
        self.trajGrabCup.addRHCartWP([0.10867544941662897, -0.4928556220711703, 1.1115715529010828])
        self.trajGrabCup.addRHCartWP([0.10824267393545398, -0.45240803263570734, 1.0964760692134503])
        self.trajGrabCup.addRHCartWP([0.12120469040222284, -0.43836063279298243, 1.0969225174809387])
        self.trajGrabCup.addRHCartWP([0.1272501926931332, -0.4388471998294017, 1.0977763079826093])

        self.trajGrabCup.addRHOrientWP([-0.2316291585632053, -0.7623882917332169, 0.6042450045564375])
        self.trajGrabCup.addRHOrientWP([-0.14571285558181804, -0.9232676106217279, 0.3554499724223333])
        self.trajGrabCup.addRHOrientWP([-0.13759279716359518, -0.9736508801139395, 0.18185759764730308])
        self.trajGrabCup.addRHOrientWP([0.0208212581992277, -0.9972797309250293, 0.07070794504934906])
        self.trajGrabCup.addRHOrientWP([0.1087330472701022, -0.9930601319391186, 0.044818509395317195])
        self.trajGrabCup.addRHOrientWP([0.10804875120890188, -0.9928585124174236, 0.05057115464825496])

        self.trajGrabCup.addPostureWP([-0.04007265239686432, -0.04007265239686432, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2766144237126975, 0.5613277829303831, 1.0122189447406513, 1.5577042110937585, 1.4883468407391849, -0.6566846935864384, -0.2042202017059911])
        self.trajGrabCup.addPostureWP([-0.04048198165032809, -0.04048198165032809, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2767139537379171, 0.3711995507250817, 0.9443563288773992, 1.5901945964948418, 1.637149246510939, -0.728673468932723, -0.13624924827024373])
        self.trajGrabCup.addPostureWP([-0.040111485332010775, -0.040111485332010775, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.26485959188580077, 0.22825283770483554, 0.9119786511961782, 1.619712922597768, 1.7109616890303816, -0.7341567675896471, -0.13653316715283456])
        self.trajGrabCup.addPostureWP([-0.04041126488088669, -0.04041126488088669, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.35452349436930974, 0.15724810167005612, 0.8052162255713683, 1.716208359958016, 1.8350970768567458, -0.77462163373844, -0.06330669049067388])
        self.trajGrabCup.addPostureWP([-0.040249248527305534, -0.040249248527305534, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.27908909321818587, 0.08692644710685907, 0.8158573514073514, 1.7425886943820597, 1.8212246719415122, -0.8888164304380365, 0.07747053323121099])
        self.trajGrabCup.addPostureWP([-0.04003492899878387, -0.04003492899878387, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2543438013482231, 0.08735540622218299, 0.8148210802232138, 1.7322744903490337, 1.804180472815585, -0.8913073159435576, 0.0834945733121769])


        # ==============================================================================================        
        self.trajPutInMicrowave = Trajectory.Trajectory("PutInMicrowave", TIME_PUT_IN_MICROWAVE)
        self.trajPutInMicrowave.setPrevTraj(self.trajGrabCup)

        # Left hand does not move
        self.trajPutInMicrowave.makeLHCartStatic(self.trajGrabCup)
        self.trajPutInMicrowave.makeLHOrientStatic(self.trajGrabCup)

        self.trajPutInMicrowave.addRHCartWP([0.13290393901668318, -0.43329868331472354, 1.0949397238274843])
        self.trajPutInMicrowave.addRHCartWP([0.18527225847571854, -0.38244573288111855, 1.1094520054189454])
        self.trajPutInMicrowave.addRHCartWP([0.224938656912424, -0.3301154261348896, 1.1159019216775588])
        self.trajPutInMicrowave.addRHCartWP([0.24189778660082983, -0.26815938143259366, 1.1204857328338538])
        self.trajPutInMicrowave.addRHCartWP([0.268167698477241, -0.2584209771905216, 1.1396434940481088])
        self.trajPutInMicrowave.addRHCartWP([0.3253788241227988, -0.23909333512118575, 1.1717743282251722])
        self.trajPutInMicrowave.addRHCartWP([0.36277713157699065, -0.22341450238565735, 1.1931700939722532])
        self.trajPutInMicrowave.addRHCartWP([0.40162129273392005, -0.2348306565511927, 1.1906979043485213])
        self.trajPutInMicrowave.addRHCartWP([0.44580366402611854, -0.11386099341078648, 1.1712545254646356])
        self.trajPutInMicrowave.addRHCartWP([0.4731953810275898, -0.10332941248132961, 1.162569166651194])

        self.trajPutInMicrowave.addRHOrientWP([-0.014952349210422919, -0.9998200609648601, 0.011673600357775012])
        self.trajPutInMicrowave.addRHOrientWP([0.21672542016365812, -0.9762126884428074, -0.0062353170049335655])
        self.trajPutInMicrowave.addRHOrientWP([0.4224104694415421, -0.9061233622029271, -0.02257980903903438])
        self.trajPutInMicrowave.addRHOrientWP([0.5183595666087359, -0.8509723623705607, -0.0845541139547354])
        self.trajPutInMicrowave.addRHOrientWP([0.5428243157194921, -0.836889970013884, 0.07040554206755197])
        self.trajPutInMicrowave.addRHOrientWP([0.5992840613687345, -0.7869147639099789, 0.1470502231548695])
        self.trajPutInMicrowave.addRHOrientWP([0.6217585155228479, -0.7588642871001055, 0.19375588284971704])
        self.trajPutInMicrowave.addRHOrientWP([0.5881027052986586, -0.8067564039048221, 0.05726528423886959])
        self.trajPutInMicrowave.addRHOrientWP([0.6203682682870011, -0.7833828939721095, -0.03813729322421932])
        self.trajPutInMicrowave.addRHOrientWP([0.4808946961840549, -0.8745964425821992, 0.06181709961335399])


        self.trajPutInMicrowave.addPostureWP([-0.039934741379565326, -0.039934741379565326, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.22264746458979268, 0.057645228429127914, 0.8192047436273743, 1.7304780739068926, 1.8001078736929355, -0.7775945795431447, 0.03011851614699232])
        self.trajPutInMicrowave.addPostureWP([-0.04016066712047841, -0.04016066712047841, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1702763776451179, 0.05434930216496875, 0.5918374087717694, 1.7779601085689638, 1.8370396447888342, -0.7756579049132685, 0.03585808834047047])
        self.trajPutInMicrowave.addPostureWP([-0.040092737697826515, -0.040092737697826515, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.11730210316452805, 0.054626522409654925, 0.3764552063521313, 1.7737125380203855, 1.8566048931632526, -0.7746961019785272, 0.035206904411013595])
        self.trajPutInMicrowave.addPostureWP([-0.04005378968284623, -0.04005378968284623, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.08060665630268302, -0.0717611621911061, 0.24755946689454936, 1.80871651784943, 1.8356163204828986, -0.7569597454783134, 0.053205366144826925])
        self.trajPutInMicrowave.addPostureWP([-0.04142935254947781, -0.04142935254947781, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.022713601599606267, -0.07837737396509815, 0.21768376401841294, 1.7689743904019695, 1.6625226626107827, -0.7973314827663038, 0.09338666454341511])
        self.trajPutInMicrowave.addPostureWP([-0.041824680098715074, -0.041824680098715074, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.2400563515030949, -0.0753116059584976, 0.15278411273039758, 1.6145398316271427, 1.6053261990012673, -0.8145803456611033, 0.12239697782675844])
        self.trajPutInMicrowave.addPostureWP([-0.041238157209725186, -0.041238157209725186, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.38535321069551726, -0.07834749998936787, 0.10978955603624256, 1.4855366360628126, 1.550662705635051, -0.8095160162636368, 0.21525847490407232])
        self.trajPutInMicrowave.addPostureWP([-0.040944863462859755, -0.040944863462859755, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5168183270124165, -0.030319531404533216, 0.10931450304789984, 1.2646431897412622, 1.6574847743599468, -0.7130376747388996, 0.3128353334588677])
        self.trajPutInMicrowave.addPostureWP([-0.038351688090164045, -0.038351688090164045, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.688726848562527, -0.19619036732424153, -0.06354041855310956, 0.8947509963708558, 1.573020397457504, -0.4895884741985058, 0.08860345270056123])
        self.trajPutInMicrowave.addPostureWP([-0.037941405297433776, -0.037941405297433776, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.8408587313815316, -0.2060178735118724, -0.05377011680265793, 0.5969284562896725, 1.4106002232384904, -0.29875989723018187, -0.09931886158312789])

        # ==============================================================================================        
        self.trajRemoveHand = Trajectory.Trajectory("RemoveHand", TIME_REMOVE_HAND)
        self.trajRemoveHand.setPrevTraj(self.trajPutInMicrowave)

        # Left hand does not move
        self.trajRemoveHand.makeLHCartStatic(self.trajPutInMicrowave)
        self.trajRemoveHand.makeLHOrientStatic(self.trajPutInMicrowave)

        self.trajRemoveHand.addRHCartWP([0.47119534358064746, -0.12449674394795462, 1.159836831346125])
        self.trajRemoveHand.addRHCartWP([0.4236115009509923, -0.14545448112794925, 1.1634881576765386])
        self.trajRemoveHand.addRHCartWP([0.3786155662877312, -0.14613241031218332, 1.1770687014321575])
        self.trajRemoveHand.addRHCartWP([0.3251714879653987, -0.17651381911299455, 1.1692815213641417])
        self.trajRemoveHand.addRHCartWP([0.26406210129290075, -0.2018332409755626, 1.1621121489676702])
        self.trajRemoveHand.addRHCartWP([0.22426097557226143, -0.24114714346888125, 1.165866457882914])
        self.trajRemoveHand.addRHCartWP([0.21119202160670944, -0.28889895041971164, 1.1595565298146817])
        self.trajRemoveHand.addRHCartWP([0.19381672984227896, -0.3398480166365939, 1.159523583173055])
        self.trajRemoveHand.addRHCartWP([0.17136140324429794, -0.3774182033406217, 1.1403083345977867])
        self.trajRemoveHand.addRHCartWP([0.17002141411721206, -0.3786171501699649, 1.1180044012874297])


        self.trajRemoveHand.addRHOrientWP([0.4765597924130374, -0.8789645612179194, 0.017665343988519454])
        self.trajRemoveHand.addRHOrientWP([0.5264107216723671, -0.8499278752053465, 0.022679485384431007])
        self.trajRemoveHand.addRHOrientWP([0.543314823688116, -0.8395287815446326, 0.0006536963635403623])
        self.trajRemoveHand.addRHOrientWP([0.5954058079106571, -0.8027154315271364, -0.03376181118454709])
        self.trajRemoveHand.addRHOrientWP([0.5622726323571884, -0.8260461616460589, -0.038694001239611656])
        self.trajRemoveHand.addRHOrientWP([0.4376709567235932, -0.8985775387074574, -0.031662921076597586])
        self.trajRemoveHand.addRHOrientWP([0.2641236270427925, -0.9639790928365276, -0.031353121245341946])
        self.trajRemoveHand.addRHOrientWP([0.13343367661766148, -0.9908371016094002, 0.020911528365385682])
        self.trajRemoveHand.addRHOrientWP([0.022357530617408774, -0.9996678278717182, 0.012820871379602009])
        self.trajRemoveHand.addRHOrientWP([0.02056695121564365, -0.9991769835537836, -0.03496220894130127])

        self.trajRemoveHand.addPostureWP([-0.03795787236691982, -0.03795787236691982, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.8276161154612388, -0.20468758025001438, -0.052421810484784996, 0.6110535908451238, 1.454599273973689, -0.29834113932268824, -0.11264101386679526])
        self.trajRemoveHand.addPostureWP([-0.03898380812312079, -0.03898380812312079, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5959423777341585, -0.09656078344036978, -0.18788375989833145, 1.0333668497681758, 1.6400080917239976, -0.3411382382138161, -0.018596759892105588])
        self.trajRemoveHand.addPostureWP([-0.03821974316543047, -0.03821974316543047, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.43396514394540964, -0.06487190146293866, -0.18350794166795167, 1.344958313487969, 1.7156873074140508, -0.3574254800129944, 0.08924681721818546])
        self.trajRemoveHand.addPostureWP([-0.03871506875783636, -0.03871506875783636, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.23927904114659593, 0.03515499942834462, -0.17591750317111565, 1.5989367377529198, 1.8549821499654466, -0.412281588802773, 0.13545650113581686])
        self.trajRemoveHand.addPostureWP([-0.038916507660601354, -0.038916507660601354, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.009361070851978855, 0.04435812125129535, -0.0868664379083671, 1.8456159934280698, 1.8521963141913125, -0.4464186133774997, 0.1657941183776347])
        self.trajRemoveHand.addPostureWP([-0.03938954609133069, -0.03938954609133069, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.13137571385260113, 0.045269490185797234, 0.06759098271111885, 1.9978778103418604, 1.8533461163869136, -0.44785088741614376, 0.16538613915612618])
        self.trajRemoveHand.addPostureWP([-0.03999600352000846, -0.03999600352000846, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1639615065156175, 0.045027732392691054, 0.2552493051422721, 1.9926966119059908, 1.8551633668501544, -0.4499569548560847, 0.1744226323361484])
        self.trajRemoveHand.addPostureWP([-0.039592400806730446, -0.039592400806730446, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.16433072866075568, 0.0453270066640155, 0.4628491612348125, 1.9786060164320538, 1.8459359115935818, -0.5344468951753123, 0.2686532438388155])
        self.trajRemoveHand.addPostureWP([-0.03930320886597833, -0.03930320886597833, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.19605295626383343, 0.04347401217448132, 0.6140987569848506, 1.9184716731071334, 1.8473021320369525, -0.5793699036514672, 0.31458955746497674])
        self.trajRemoveHand.addPostureWP([-0.03938577341106948, -0.03938577341106948, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.21332751531771438, 0.03178948423978515, 0.6123939704486326, 1.8562822842499185, 1.8484632177857667, -0.5847728926959009, 0.30138086134441316])

        # ==============================================================================================        
        self.trajGoToIdle = Trajectory.Trajectory("GoToIdle", TIME_GO_TO_IDLE)
        self.trajGoToIdle.setPrevTraj(self.trajRemoveHand)

        # Left hand does not move
        self.trajGoToIdle.makeLHCartStatic(self.trajRemoveHand)
        self.trajGoToIdle.makeLHOrientStatic(self.trajRemoveHand)

        self.trajGoToIdle.addRHCartWP([0.17419046827681037, -0.36252565961673056, 1.102183976132919])
        self.trajGoToIdle.addRHCartWP([0.15492926562539974, -0.45033170309415005, 1.2054842715955094])
        self.trajGoToIdle.addRHCartWP([0.08877847893119784, -0.5115182283337758, 1.2375283142809226])
        self.trajGoToIdle.addRHCartWP([0.05955418636025552, -0.5501719533324743, 1.1835005845242361])
        self.trajGoToIdle.addRHCartWP([0.010341458616731504, -0.5849810996032294, 1.11955087467655])
        self.trajGoToIdle.addRHCartWP([-0.00022585060512272394, -0.55469520487658, 1.017549376137806])
        self.trajGoToIdle.addRHCartWP([0.007693298232455261, -0.5104389678245014, 0.9583591709332557])
        self.trajGoToIdle.addRHCartWP([0.05546004718334695, -0.36591699369660236, 0.888635526573475])
        self.trajGoToIdle.addRHCartWP([0.07596186979930318, -0.2989159868424402, 0.8823545117230572])
        self.trajGoToIdle.addRHCartWP([0.06384941737139532, -0.3014666346540123, 0.8698680026168045])

        self.trajGoToIdle.addRHOrientWP([0.015633725856431437, -0.994873607593614, -0.09991041752246661])
        self.trajGoToIdle.addRHOrientWP([-0.30476951371096606, -0.8351165127643699, 0.4579257075341581])
        self.trajGoToIdle.addRHOrientWP([-0.43914283034907864, -0.33589465692601544, 0.8332636761563262])
        self.trajGoToIdle.addRHOrientWP([-0.27254509562302237, -0.295256369486552, 0.9157198518812717])
        self.trajGoToIdle.addRHOrientWP([-0.33239716151882714, -0.4626583974668991, 0.8218633306502897])
        self.trajGoToIdle.addRHOrientWP([-0.301606231590617, -0.6952742341590504, 0.6524012725158137])
        self.trajGoToIdle.addRHOrientWP([-0.006782725254935766, -0.8078381000736575, 0.589365420352687])
        self.trajGoToIdle.addRHOrientWP([0.7268826951413586, -0.5409600405272441, 0.42308838563212425])
        self.trajGoToIdle.addRHOrientWP([0.7968216104623959, -0.025876562816040706, 0.603660272501795])
        self.trajGoToIdle.addRHOrientWP([0.8477482256589355, -0.003394510486248027, 0.5303879930679855])

        self.trajGoToIdle.addPostureWP([-0.03947795753465285, -0.03947795753465285, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.194098347164122, -0.04412554947889996, 0.6060796177654195, 1.8362224356553405, 1.8217114355140576, -0.5895714519538124, 0.2753564101967997])
        self.trajGoToIdle.addPostureWP([-0.03964478341623607, -0.03964478341623607, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.15754792983893193, 0.2824090763425676, 0.7876095223751777, 1.896649162195286, 1.525080227730949, -0.46797362257559655, 0.16185815637887202])
        self.trajGoToIdle.addPostureWP([-0.03945344199055476, -0.03945344199055476, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.15869585920464954, 0.35132932071964385, 1.110321697392817, 1.8567488166428108, 0.9404716193637824, -0.48880668524203863, 0.1642372659266582])
        self.trajGoToIdle.addPostureWP([-0.03949456963443166, -0.03949456963443166, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1541654669202991, 0.33011804690452123, 1.2268223913290617, 1.6359644404907407, 0.6187204774536202, -0.4872612739098581, 0.16344083551813984])
        self.trajGoToIdle.addPostureWP([-0.039504490141409214, -0.039504490141409214, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.15145346651742836, 0.34654269047105934, 1.387455292927428, 1.3446364180252888, 0.621905440940878, -0.4935880107611833, 0.2110508784252269])
        self.trajGoToIdle.addPostureWP([-0.039939300378912045, -0.039939300378912045, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.15339811838047496, 0.24407420737035557, 1.3825703401694656, 1.1175442000709033, 0.6013614849569962, -0.49820256777486827, 0.2017965708263541])
        self.trajGoToIdle.addPostureWP([-0.03981245992110429, -0.03981245992110429, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1521525122730378, 0.1474224108064928, 1.3229050911171845, 1.0257604132274654, 0.3079475225042759, -0.5051856203074209, 0.18991961663950238])
        self.trajGoToIdle.addPostureWP([-0.04017315310876321, -0.04017315310876321, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.3570026428275251, 0.01356712974127779, 0.6231532977781696, 1.0562737153459774, 0.11498759800385559, -0.40836581299808655, 0.0719337276506664])
        self.trajGoToIdle.addPostureWP([-0.04063173887102727, -0.04063173887102727, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.41337916798126034, 0.05811741782500938, 0.23706704531611159, 1.0931952745322413, -0.05908222522713845, -0.11601619503236027, -0.17303188027780114])
        self.trajGoToIdle.addPostureWP([-0.04042140933987992, -0.04042140933987992, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.40528670918178583, 0.05911915951681042, 0.2518362818876945, 1.019561855790158, -0.11159989370755907, -0.13012478004629885, -0.15493141215426787])

    def createFSM(self):
        # define the states
        goToReadyState = TrajectoryState(self.dreamerInterface, self.trajGoToReady)
        grabCupState = TrajectoryState(self.dreamerInterface, self.trajGrabCup)
        putInMicrowaveState = TrajectoryState(self.dreamerInterface, self.trajPutInMicrowave)
        removeHandState = TrajectoryState(self.dreamerInterface, self.trajRemoveHand)
        goToIdleState = TrajectoryState(self.dreamerInterface, self.trajGoToIdle)

        enablePowerGraspState = EnablePowerGraspState(self.dreamerInterface)
        disablePowerGraspState = DisablePowerGraspState(self.dreamerInterface)

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReady", goToReadyState, 
                transitions={'done':'GrabCup',
                             'exit':'exit'})
            smach.StateMachine.add("GrabCup", grabCupState, 
                transitions={'done':'EnablePowerGraspState',
                             'exit':'exit'})
            smach.StateMachine.add("EnablePowerGraspState", enablePowerGraspState, 
                transitions={'done':'PutInMicrowaveState',
                             'exit':'exit'})
            smach.StateMachine.add("PutInMicrowaveState", putInMicrowaveState, 
                transitions={'done':'DisablePowerGraspState',
                             'exit':'exit'})
            smach.StateMachine.add("DisablePowerGraspState", disablePowerGraspState, 
                transitions={'done':'RemoveHandState',
                             'exit':'exit'})
            smach.StateMachine.add("RemoveHandState", removeHandState, 
                transitions={'done':'GoToIdle',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdle", goToIdleState, 
                transitions={'done':'exit',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 11 behavior.
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

        print "Demo 11 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo11_SaltShake', anonymous=True)
    demo = Demo11_Microwave()
    demo.run()

    
