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
TIME_GRAB_SALT = 3.0
TIME_SHAKE_SALT = 6.0
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

class Demo10_SaltShake:
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
        
        self.trajGoToReady.addRHCartWP([0.05914076433495025, -0.26057331586469357, 0.8075152343995867])
        self.trajGoToReady.addRHCartWP([0.023313741346565105, -0.3573590975417546, 0.8907716749859689])
        self.trajGoToReady.addRHCartWP([0.007323613968634814, -0.5058796562308474, 0.9987523903297643])
        self.trajGoToReady.addRHCartWP([0.05108178190737086, -0.6077126537849552, 1.1437628561097104])
        self.trajGoToReady.addRHCartWP([0.07989653965489718, -0.6394815090741853, 1.3530833331358687])
        self.trajGoToReady.addRHCartWP([0.15679462139090777, -0.5414846920174312, 1.398644666928851])
        self.trajGoToReady.addRHCartWP([0.14559806210640328, -0.5100454714857916, 1.342764803613533])


        self.trajGoToReady.addRHOrientWP([0.9971756062597187, -0.027385086491198, 0.06993473613614432])
        self.trajGoToReady.addRHOrientWP([0.9525563922936429, 0.22360444317139683, 0.2064881897216629])
        self.trajGoToReady.addRHOrientWP([0.9741246098559916, 0.21917378212320426, 0.05517334231966054])
        self.trajGoToReady.addRHOrientWP([0.92758361985272, 0.2618971829551492, 0.26645542542999945])
        self.trajGoToReady.addRHOrientWP([0.39225456916415047, 0.5636068097964206, 0.7269688555371187])
        self.trajGoToReady.addRHOrientWP([-0.28727863887316335, 0.46856710939457835, 0.8354135787984248])
        self.trajGoToReady.addRHOrientWP([-0.19034100158747316, -0.150262853599855, 0.9701501831895443])


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


        self.trajGoToReady.addPostureWP([0.02402533370499574, 0.02402533370499574,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1307406592182293, 0.09014266732964009, -0.01879322218479341, 0.5029941957856777, 0.07019071232775005, -0.25646144756424455, -0.04379860354646468])
        self.trajGoToReady.addPostureWP([0.025058277189434867, 0.025058277189434867,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.5142387129923343, 0.3719960146291461, -0.030328690706992303, 1.0650635319221833, 0.06354609022502447, -0.2610737510253198, -0.04870693306669968])
        self.trajGoToReady.addPostureWP([0.024908976608228276, 0.024908976608228276,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.83126035627492, 0.8883014692178023, -0.010482398645718004, 1.1560798699364918, 0.5172676305529517, -0.25787237856424305, -0.05127779726762061])
        self.trajGoToReady.addPostureWP([0.024694975155595784, 0.024694975155595784,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -1.0218030949373402, 0.8810019756995155, 0.782193061567671, 1.249242514103173, 0.2615654282049771, -0.2658595252341787, -0.07047616930291978])
        self.trajGoToReady.addPostureWP([0.02407540362842636, 0.02407540362842636,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6675828257176796, 0.9596251345149525, 1.4203097900780723, 1.3378972691068456, 0.2326991527127683, -0.0841834575402748, -0.28045276926782703])
        self.trajGoToReady.addPostureWP([0.024420103183194798, 0.024420103183194798,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6232408095125116, 1.0208026606134355, 1.298161758278034, 1.7307862806625909, 0.8660955010828015, -0.05733352356855179, -0.2998289890966412])
        self.trajGoToReady.addPostureWP([0.024602563528443925, 0.024602563528443925,
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.5906990442416915, 0.832621035619301, 1.1720080777390485, 1.9063970964812296, 1.3207550061877573, -0.3657973772415582, 0.02523566386067858])

        # ==============================================================================================
        self.trajGrabSalt = Trajectory.Trajectory("GrabSalt", TIME_GRAB_SALT)
        self.trajGrabSalt.setPrevTraj(self.trajGoToReady)

        # Left hand does not move
        self.trajGrabSalt.makeLHCartStatic(self.trajGoToReady)
        self.trajGrabSalt.makeLHOrientStatic(self.trajGoToReady)
        
        # Specify the waypoints
        self.trajGrabSalt.addRHCartWP([0.1368682530366739, -0.4980039231047034, 1.3191871082797064])
        self.trajGrabSalt.addRHCartWP([0.13635589186985528, -0.4709988159760668, 1.2713945875976562])
        self.trajGrabSalt.addRHCartWP([0.14618931808219662, -0.43832890094635923, 1.1959226641646907])
        self.trajGrabSalt.addRHCartWP([0.23122834991618751, -0.3759418639694284, 1.1380250551488178])
        self.trajGrabSalt.addRHCartWP([0.24638273806268182, -0.3304903646012086, 1.1129094300963527])
        self.trajGrabSalt.addRHCartWP([0.2651442259701546, -0.30586436436275327, 1.1084635692483154])

        self.trajGrabSalt.addRHOrientWP([-0.08273772643658983, -0.20757692812633197, 0.9747134386750496])
        self.trajGrabSalt.addRHOrientWP([-0.058198595253024656, -0.6970632679150955, 0.7146437742218865])
        self.trajGrabSalt.addRHOrientWP([-0.2753488365356643, -0.8211936348253884, 0.49982400136520305])
        self.trajGrabSalt.addRHOrientWP([0.1217001579857854, -0.9840660672490058, 0.1296265668576858])
        self.trajGrabSalt.addRHOrientWP([0.09748668697686975, -0.9939307964130979, -0.050969773433858605])
        self.trajGrabSalt.addRHOrientWP([0.061417388747668604, -0.9958366630324049, -0.0673590745178568])

        self.trajGrabSalt.addPostureWP([0.024744075530812384, 0.024744075530812384, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6602731008864706, 0.7636531673323284, 1.1499816915255239, 1.9680001903569746, 1.3350327338045107, -0.4098808309820463, 0.07155096651358149])
        self.trajGrabSalt.addPostureWP([0.02413043876209492, 0.02413043876209492, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.6306532370748459, 0.640196501475271, 0.9908681363733606, 2.0398781744713532, 1.7984430223746184, -0.4195716561325461, 0.10938188714847934])
        self.trajGrabSalt.addPostureWP([0.023954504636184593, 0.023954504636184593, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.2528283044280894, 0.241939899484367, 0.8466032549265826, 2.0229372230617706, 1.5757444609320181, -0.4533822239864415, 0.15511920253576847])
        self.trajGrabSalt.addPostureWP([0.024084409232604593, 0.024084409232604593, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.20400979762922297, 0.4250593848966316, 0.3112979683032229, 1.8255756906977358, 1.9461599805027823, -0.3014572383706713, 0.059457533050168286])
        self.trajGrabSalt.addPostureWP([0.024299183898934384, 0.024299183898934384, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.09254415118023684, 0.14865494896562292, 0.31045606636076456, 1.8108398106197838, 1.8654128823849236, -0.34956477431032085, 0.2563381314014863])
        self.trajGrabSalt.addPostureWP([0.024468556080276375, 0.024468556080276375, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.007657017204115373, 0.022047159014570564, 0.31124879227806, 1.7816278995681702, 1.7423174919204645, -0.34755920471176044, 0.2550035649062545])
       

        # ==============================================================================================        
        self.trajShakeSalt = Trajectory.Trajectory("ShakeSalt", TIME_SHAKE_SALT)
        self.trajShakeSalt.setPrevTraj(self.trajGrabSalt)

        # Left hand does not move
        self.trajShakeSalt.makeLHCartStatic(self.trajGrabSalt)
        self.trajShakeSalt.makeLHOrientStatic(self.trajGrabSalt)

        self.trajShakeSalt.addRHCartWP([0.26530929720040136, -0.30083727820053385, 1.1039035360509957])
        self.trajShakeSalt.addRHCartWP([0.26973116642617023, -0.2974883314074139, 1.1664597213564902])
        self.trajShakeSalt.addRHCartWP([0.2806733449895382, -0.24924805932297583, 1.242770610137349])
        self.trajShakeSalt.addRHCartWP([0.2576813400454686, -0.18651466939669212, 1.249596610452429])
        self.trajShakeSalt.addRHCartWP([0.2717514908162565, -0.17879732577599416, 1.298421765064597])
        self.trajShakeSalt.addRHCartWP([0.2749482427304945, -0.16218663199617478, 1.248597572483494])
        self.trajShakeSalt.addRHCartWP([0.27604983426394203, -0.19962641313739218, 1.2960023317257137])
        self.trajShakeSalt.addRHCartWP([0.2903096963359411, -0.2518796077116205, 1.28375926671974])
        self.trajShakeSalt.addRHCartWP([0.2980760709201117, -0.3295381677205583, 1.2575004522057565])
        self.trajShakeSalt.addRHCartWP([0.2938456406671425, -0.37979268060529, 1.2151467822507012])
        self.trajShakeSalt.addRHCartWP([0.28930670828500504, -0.35714308462050814, 1.1505123053308979])
        self.trajShakeSalt.addRHCartWP([0.28922772680421394, -0.33726068412436305, 1.1131075743023073])
        self.trajShakeSalt.addRHCartWP([0.28608851979277206, -0.33257879624066533, 1.1065788027324677])

        self.trajShakeSalt.addRHOrientWP([0.06659717284493136, -0.9968015403400663, -0.044175850243466756])
        self.trajShakeSalt.addRHOrientWP([0.07068124423764442, -0.991516700357322, 0.1090815961816979])
        self.trajShakeSalt.addRHOrientWP([-0.0028744275985558847, -0.8335474395150876, 0.5524404074140687])
        self.trajShakeSalt.addRHOrientWP([-0.1358197957790845, -0.13791068522379735, 0.9810879807517932])
        self.trajShakeSalt.addRHOrientWP([-0.6490895016646091, 0.5707491701226808, 0.5029196791069726])
        self.trajShakeSalt.addRHOrientWP([-0.557148469926602, 0.5747612983373852, 0.5993621879898408])
        self.trajShakeSalt.addRHOrientWP([-0.47291157140882734, 0.49545542841100493, 0.7286141393671284])
        self.trajShakeSalt.addRHOrientWP([-0.22842305046885827, 0.26275729862264263, 0.9374334707247404])
        self.trajShakeSalt.addRHOrientWP([-0.05951741242087269, -0.08565984955646297, 0.9945451562360997])
        self.trajShakeSalt.addRHOrientWP([-0.061927535110660605, -0.5941690202822528, 0.8019527141496222])
        self.trajShakeSalt.addRHOrientWP([0.061160377708257746, -0.8670569220082821, 0.49444079746326264])
        self.trajShakeSalt.addRHOrientWP([0.15771779711212583, -0.9636644636800336, 0.2155826939120387])
        self.trajShakeSalt.addRHOrientWP([0.27370062774256604, -0.9617991235647599, 0.005514733301354368])
        
        self.trajShakeSalt.addPostureWP([0.02436080008470486, 0.02436080008470486, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.009197597279545592, 0.003946761468741407, 0.3067225627169007, 1.7742975714881093, 1.696410190442201, -0.3576206152918874, 0.23577468382741087])
        self.trajShakeSalt.addPostureWP([0.02450507257936768, 0.02450507257936768, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.06704167487429545, 0.004358356402465086, 0.30898236585317645, 1.9360960169335657, 1.6094318357430304, -0.39329795545184015, 0.3167723598976957])
        self.trajShakeSalt.addPostureWP([0.024361678107943596, 0.024361678107943596, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.22234622280244382, 0.004548223545309593, 0.13071522011414413, 2.0817017281696133, 1.1418970725651827, -0.4552467192731869, 0.335930673724112])
        self.trajShakeSalt.addPostureWP([0.024332337243309566, 0.024332337243309566, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.16698047214390493, -0.009707208699479875, -0.12402218104724971, 2.182201689193785, 0.1489575444963377, -0.5712434934364764, 0.09595690564229147])
        self.trajShakeSalt.addPostureWP([0.025468511543542877, 0.025468511543542877, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.6216710691442365, 0.6715737490501017, -0.6437964700413162, 2.1247705467775284, -0.1441990611776954, -0.3057524639307719, -0.3092821028996973])
        self.trajShakeSalt.addPostureWP([0.025403325829391032, 0.025403325829391032, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5437735484531676, 0.6326916011583551, -0.744919211391087, 2.047329658353664, -0.14467963269495024, -0.31590819398726516, -0.29829881611639514])
        self.trajShakeSalt.addPostureWP([0.025538985616602845, 0.025538985616602845, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.4721646511992432, 0.5711817561108634, -0.42322977595535693, 2.126791779640995, -0.013012518067382737, -0.41412143468303314, -0.271159551975299])
        self.trajShakeSalt.addPostureWP([0.026384310012505203, 0.026384310012505203, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.34055061750814686, 0.5258918238208286, -0.16127303190027792, 2.059073434254529, 0.29760181111300077, -0.47517876689080657, -0.20144048668789935])
        self.trajShakeSalt.addPostureWP([0.02628160068665444, 0.02628160068665444, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.22413955315509168, 0.5009679278493124, 0.12370844623095786, 1.9192989590033398, 0.6393347914400501, -0.4667458060549741, -0.19479962992865535])
        self.trajShakeSalt.addPostureWP([0.024368664789607912, 0.024368664789607912, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.21897583626527597, 0.280341408527275, 0.40379423091638394, 1.7890040208130233, 0.9422130928768778, -0.526108764888531, -0.13471860411097075])
        self.trajShakeSalt.addPostureWP([0.024910770843457775, 0.024910770843457775, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.14219838746455393, 0.12616914086344647, 0.4142910286228933, 1.7289071693320457, 1.2174849707601356, -0.5484305641891236, -0.07975075160368959])
        self.trajShakeSalt.addPostureWP([0.024768694239681307, 0.024768694239681307, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.12019693214181422, 0.03166299202738259, 0.41362450616755464, 1.6695271535542155, 1.426683337395159, -0.5971375938632858, -0.009203734869275617])
        self.trajShakeSalt.addPostureWP([0.024733771343043814, 0.024733771343043814, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.10360896498976005, 0.017635370325832966, 0.40886134101865207, 1.6695959043723732, 1.6846222090792375, -0.6784336408318479, 0.2678733444053529])

        # ==============================================================================================        
        self.trajGoToIdle = Trajectory.Trajectory("GoToIdle", TIME_GO_TO_IDLE)
        self.trajGoToIdle.setPrevTraj(self.trajShakeSalt)

        # Left hand does not move
        self.trajGoToIdle.makeLHCartStatic(self.trajShakeSalt)
        self.trajGoToIdle.makeLHOrientStatic(self.trajShakeSalt)

        self.trajGoToIdle.addRHCartWP([0.2855510093731542, -0.33275298782210605, 1.105961513583146])
        self.trajGoToIdle.addRHCartWP([0.3118565433451554, -0.38098180411775384, 1.2387201547139597])
        self.trajGoToIdle.addRHCartWP([0.2366343156163923, -0.46457871689022895, 1.3248977218365872])
        self.trajGoToIdle.addRHCartWP([0.17956164810447026, -0.4954682566410858, 1.3118143241780265])
        self.trajGoToIdle.addRHCartWP([0.06542445018341615, -0.5226600395200283, 1.2537559687475741])
        self.trajGoToIdle.addRHCartWP([0.006151047301281022, -0.5536230447356679, 1.1648556970561526])
        self.trajGoToIdle.addRHCartWP([-0.023826182717746464, -0.5166405657747549, 0.9801765656331256])
        self.trajGoToIdle.addRHCartWP([0.04839620752417691, -0.3770701558098519, 0.8928547633193239])
        self.trajGoToIdle.addRHCartWP([0.03899699424187923, -0.3058468309579561, 0.85001141642397])

        self.trajGoToIdle.addRHOrientWP([0.2775168852086292, -0.9606262811152241, 0.013473175380545727])
        self.trajGoToIdle.addRHOrientWP([0.057808389189858256, -0.8613323212567038, 0.5047423327776388])
        self.trajGoToIdle.addRHOrientWP([-0.4645251560931721, -0.6643854342237623, 0.5854983980745948])
        self.trajGoToIdle.addRHOrientWP([-0.16082968100398085, -0.7629787193685621, 0.6260968675044355])
        self.trajGoToIdle.addRHOrientWP([-0.3685936361128062, -0.7163190552806472, 0.5924742546803035])
        self.trajGoToIdle.addRHOrientWP([-0.48543265151711873, -0.6949795783147527, 0.5304323958493752])
        self.trajGoToIdle.addRHOrientWP([-0.2220946874607929, -0.9077174403305266, 0.355981738747214])
        self.trajGoToIdle.addRHOrientWP([0.5586839928649725, -0.7564737006830804, 0.340029022719086])
        self.trajGoToIdle.addRHOrientWP([0.9484107547164847, -0.04658272139778326, 0.31360339667370374])
      

        self.trajGoToIdle.addPostureWP([0.024606146828888193, 0.024606146828888193, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.10210171461772798, 0.016852514207610723, 0.4102637558684599, 1.6690501292036815, 1.674345801931866, -0.6858570043924165, 0.25368377222916494])
        self.trajGoToIdle.addPostureWP([0.024444179054270782, 0.024444179054270782, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.3523642258996476, 0.2396121699960159, 0.4261238024554756, 1.7558932480532394, 1.3976591892112387, -0.6615721654432645, 0.11867147916846761])
        self.trajGoToIdle.addPostureWP([0.02472272940815622, 0.02472272940815622, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.4862750309398737, 0.36942646700645854, 0.7752889236138033, 1.8501925268508197, 1.2081106461829472, -0.6378789346239732, 0.11439103213556615])
        self.trajGoToIdle.addPostureWP([0.02428527940360682, 0.02428527940360682, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.11154420308839788, 0.5405581360437428, 0.9219763400763243, 1.8815433096177934, 1.6751140122937183, -0.805528931535813, 0.3740307122838615])
        self.trajGoToIdle.addPostureWP([0.02397392167662364, 0.02397392167662364, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.37839745984804557, 0.4122913649664896, 1.2578340311798042, 1.9001112868960444, 1.7290745039433773, -0.6226387276145056, 0.08318963871212404])
        self.trajGoToIdle.addPostureWP([0.023473256977919715, 0.023473256977919715, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.4620702596514896, 0.3125594210774841, 1.2878843941925593, 1.6252925281205395, 1.6008896922525346, -0.5364521414711813, -0.053656654558083716])
        self.trajGoToIdle.addPostureWP([0.023645138607220822, 0.023645138607220822, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.4755215172061432, 0.25872259373613504, 0.9470806144888511, 1.0956591778990117, 1.3276363707632581, -0.5195520726287962, -0.043247681144563746])
        self.trajGoToIdle.addPostureWP([0.023957246358055746, 0.023957246358055746, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.37890285379340527, 0.08905003854511831, 0.5235219585785394, 1.0797479704800705, 0.5944826264098443, -0.42464485268665564, -0.1294609022888612])
        self.trajGoToIdle.addPostureWP([0.023637855598762605, 0.023637855598762605, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.35589626709932876, 0.05553932690495682, 0.29235494805813456, 0.9145562780667971, -0.13903281815065885, -0.18789934824279428, -0.33216257045153824])
  
    def createFSM(self):
        # define the states
        goToReadyState = TrajectoryState(self.dreamerInterface, self.trajGoToReady)
        grabSaltState = TrajectoryState(self.dreamerInterface, self.trajGrabSalt)
        shakeSaltState = TrajectoryState(self.dreamerInterface, self.trajShakeSalt)
        goToIdleState = TrajectoryState(self.dreamerInterface, self.trajGoToIdle)

        enablePowerGraspState = EnablePowerGraspState(self.dreamerInterface)
        disablePowerGraspState = DisablePowerGraspState(self.dreamerInterface)

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("GoToReady", goToReadyState, 
                transitions={'done':'GrabSalt',
                             'exit':'exit'})
            smach.StateMachine.add("GrabSalt", grabSaltState, 
                transitions={'done':'EnablePowerGraspState',
                             'exit':'exit'})
            smach.StateMachine.add("EnablePowerGraspState", enablePowerGraspState, 
                transitions={'done':'ShakeSaltState',
                             'exit':'exit'})
            smach.StateMachine.add("ShakeSaltState", shakeSaltState, 
                transitions={'done':'DisablePowerGraspState',
                             'exit':'exit'})
            smach.StateMachine.add("DisablePowerGraspState", disablePowerGraspState, 
                transitions={'done':'GoToIdle',
                             'exit':'exit'})
            smach.StateMachine.add("GoToIdle", goToIdleState, 
                transitions={'done':'exit',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 10 behavior.
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

        print "Demo 10 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo10_SaltShake', anonymous=True)
    demo = Demo10_SaltShake()
    demo.run()

    
