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
TRAJECTORY_EXECUTION_TIME = 30.0

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
    def __init__(self):
        self.dreamerInterface = DreamerInterface.DreamerInterface(enableUserPrompts = ENABLE_USER_PROMPTS, useQuaternionControl = True)

    def createTrajectories(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        self.traj = Trajectory.Trajectory("TestTraj", TRAJECTORY_EXECUTION_TIME)

        # These are the initial values as specified in the YAML ControlIt! configuration file
        self.traj.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        self.traj.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.setInitRHOrientWP([0, 1.0, 0.0, 0.0])
        self.traj.setInitLHOrientWP([0, 1.0, 0.0, 0.0])
        self.traj.setInitPostureWP(DEFAULT_POSTURE)
        
        self.traj.addRHCartWP([0.11440977698673853, -0.21487368057442519, 0.7982043300411712])
        self.traj.addRHCartWP([0.3357871543607279, -0.2572270048400681, 0.9481393747873456])
        self.traj.addRHCartWP([0.3850771669176694, -0.22533054448181658, 1.1672272305873348])
        self.traj.addRHCartWP([0.374891863626203, -0.22037273728765608, 1.2131787525872904])
        self.traj.addRHCartWP([0.3609855278666004, -0.16010561908072898, 1.239309757959955])
        self.traj.addRHCartWP([0.3490976730537494, -0.12326881859355178, 1.235514224856111])
        self.traj.addRHCartWP([0.36268017250708506, -0.19537676306022367, 1.2256005019048053])
        self.traj.addRHCartWP([0.3642457078547365, -0.21475222170444602, 1.235107822424214])
        self.traj.addRHCartWP([0.37357992796049794, -0.24635649840579063, 1.2789401591611034])
        self.traj.addRHCartWP([0.36938736870371963, -0.2572373124536631, 1.2586115421754145])
        self.traj.addRHCartWP([0.34162395898807757, -0.3243488187825392, 1.197375956586867])
        self.traj.addRHCartWP([0.32993520750605876, -0.41566612990560275, 1.1224301533374954])
        self.traj.addRHCartWP([0.3516944194458634, -0.2457387183707161, 1.1632917114162722])
        self.traj.addRHCartWP([0.34667413459972696, -0.23076672070707402, 1.176643770151852])
        self.traj.addRHCartWP([0.3466309717246493, -0.2323027579763726, 1.1758978934187616])
        self.traj.addRHCartWP([0.3408442534577974, -0.2328361305546918, 1.1598141270279385])
        self.traj.addRHCartWP([0.33660490132867465, -0.23364478448945655, 1.1304870139383925])
        self.traj.addRHCartWP([0.2472825777588422, -0.2540613083891057, 1.084261626115514])
        self.traj.addRHCartWP([0.22731556861967298, -0.27932968652556245, 0.9757506423315835])
        self.traj.addRHCartWP([0.1463533267957461, -0.2662216096416309, 0.8668232604814613])
        self.traj.addRHCartWP([0.04921384119767269, -0.25734160405048534, 0.8140582700232945])
        self.traj.addRHCartWP([0.017481865427828074, -0.24786765175997175, 0.7949944653609655])

        self.traj.addRHOrientWP([-0.0050821366730603905, 0.9729947482215079, 0.04818537408435469, 0.22568465066227156])
        self.traj.addRHOrientWP([0.07899857264972625, 0.8152395705934258, 0.1681977094973707, 0.5485008647010421])
        self.traj.addRHOrientWP([0.07366365261276137, 0.5397689418239006, 0.1516730136559514, 0.8247535708650037])
        self.traj.addRHOrientWP([-0.07623082909357848, 0.5010152973967267, 0.0028003709449283425, 0.8620700031857889])
        self.traj.addRHOrientWP([-0.4785235232373017, 0.6655823637898206, -0.07950061854573602, 0.5671816343741748])
        self.traj.addRHOrientWP([0.6311969508475875, -0.6688459342645839, 0.04235183031610037, -0.39043802059650917])
        self.traj.addRHOrientWP([-0.36482472352381573, 0.6813968808904257, -0.08624853961061045, 0.628619440706066])
        self.traj.addRHOrientWP([-0.19746535401984347, 0.46232629232089384, 0.006911771162462729, 0.8644154445693524])
        self.traj.addRHOrientWP([-0.11773134664595693, 0.10539179736247357, 0.05455876550624124, 0.9859286181930575])
        self.traj.addRHOrientWP([0.17111391200224801, 0.24659098575870417, 0.22009871760210054, 0.9281537961845269])
        self.traj.addRHOrientWP([0.35702598657263585, 0.3792267819197045, 0.29361512616453284, 0.8015669968717813])
        self.traj.addRHOrientWP([0.38335182647254634, 0.5282071829705617, 0.3383061368435431, 0.6779288360679638])
        self.traj.addRHOrientWP([-0.04495889066784156, 0.38014842760599066, 0.08648474524280969, 0.9197751138071176])
        self.traj.addRHOrientWP([-0.10535191730268513, 0.3438735148932308, 0.01888725602183363, 0.9328961629439736])
        self.traj.addRHOrientWP([-0.09960939928059132, 0.6287876574885275, 0.01991445048609333, 0.7709133959316619])
        self.traj.addRHOrientWP([-0.09233977041008325, 0.8019504003972171, 0.03683148626640397, 0.589060577294699])
        self.traj.addRHOrientWP([-0.09384747176018045, 0.886536412066043, 0.023235632825698255, 0.45244441370457733])
        self.traj.addRHOrientWP([-0.08771150961949001, 0.615549388924391, 0.04552260728752244, 0.7818780807138827])
        self.traj.addRHOrientWP([-0.041901054651243934, 0.763790440577662, 0.01967697733088622, 0.643802206476814])
        self.traj.addRHOrientWP([-0.07749581398436217, 0.8915852573719573, -0.02591389360169652, 0.4454195749742953])
        self.traj.addRHOrientWP([-0.08462654596399091, 0.9545381039566313, -0.05286308264353075, 0.28089295168534784])
        self.traj.addRHOrientWP([-0.049312751082445885, 0.9705180444686836, -0.07717913868849326, 0.22294922850879156])

        # left arm does not move
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.traj.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])
        self.traj.addLHOrientWP([0.0, 1.0, 0.0, 0.0])


        self.traj.addPostureWP([0.010800957125673784, 0.010800957125673784, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.10192136242604619, -0.0037555561402176987, 0.009475272182110356, 0.21396388487530393, -0.10326417310013992, 0.16211038710437, -0.014869287921755407])
        self.traj.addPostureWP([0.011033794516645542, 0.011033794516645542, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.3236929611293482, 0.10989571674067494, -0.05810738814683512, 0.7684484044351282, -0.2493921683815889, 0.13260743652136206, -0.05034220975507994])
        self.traj.addPostureWP([0.011310962012624905, 0.011310962012624905, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.42710658337680857, 0.11682914389082602, -0.0976733380256697, 1.4518818496018733, -0.1762415872202414, 0.13174637932335304, -0.05144319687145419])
        self.traj.addPostureWP([0.01163985951525283, 0.01163985951525283, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.45273695898455824, 0.11392755735129304, -0.09540499393720576, 1.5949807347676488, 0.23422134815003304, 0.08111177769435629, 0.005955067714578424])
        self.traj.addPostureWP([0.01123490576585092, 0.01123490576585092, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.49908873849526036, 0.1108704546368271, -0.30420332451899923, 1.6786974742940757, 1.201537429915331, -0.4319560115377949, 0.19261512492909272])
        self.traj.addPostureWP([0.010868473665071541, 0.010868473665071541, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5574634584134629, 0.2004822436021976, -0.5417831766570335, 1.6807798035622674, 1.6894812628902882, -0.436148709980282, 0.19087828810806806])
        self.traj.addPostureWP([0.011794918815107318, 0.011794918815107318, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.45810519462057103, 0.1698290632899791, -0.23028945974074297, 1.6643417929310291, 1.000197279872617, -0.4022317846788922, 0.1711515124373324])
        self.traj.addPostureWP([0.011310170774132832, 0.011310170774132832, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.4612985179518125, 0.1423095584130234, -0.13241076948904307, 1.6755985190967198, 0.4638816228911934, 0.07460407408824596, -0.08006856401590766])
        self.traj.addPostureWP([0.01123325997926735, 0.01123325997926735, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5701759200193387, 0.08351076873556262, 0.03326140866928074, 1.6797075123944551, 0.3887160859936074, 0.65641337276969, -0.3425579390725185])
        self.traj.addPostureWP([0.01172391694469394, 0.01172391694469394, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.5152602191730916, 0.08853389493145981, 0.06752766082942639, 1.6763622307290214, -0.37756677491948326, 0.5025889921208758, -0.17950616015454546])
        self.traj.addPostureWP([0.012317224322888834, 0.012317224322888834, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.3597467317730106, 0.08842852458143674, 0.30922805761273975, 1.6486215757991927, -0.774027407206302, 0.4848668236280546, -0.19570797079253693])
        self.traj.addPostureWP([0.011516426769832661, 0.011516426769832661, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.3038164897788758, 0.2717770303615966, 0.4159800268502189, 1.3079161245144622, -0.8282187614717941, 0.4943535925035672, -0.1816989860718934])
        self.traj.addPostureWP([0.011371685616762576, 0.011371685616762576, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.30217973367373846, 0.09887119484059124, 0.00984373146200727, 1.598912470212718, 0.12188421076340977, 0.4768956091421255, -0.21057954198801118])
        self.traj.addPostureWP([0.011336661048109107, 0.011336661048109107, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.30161129179774243, 0.05273950715922997, 0.003824740344451906, 1.6603977958756082, 0.26034867628782576, 0.47716458391138616, -0.20567344880773789])
        self.traj.addPostureWP([0.011160769116534137, 0.011160769116534137, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.30046235083819567, 0.05344450516325459, 0.00852799039616671, 1.6579794070708749, 0.18409465998154087, -0.1717806028509987, -0.13874178474417623])
        self.traj.addPostureWP([0.011637701424387206, 0.011637701424387206, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.2611399770984611, 0.05354979588474899, 0.009546909227140148, 1.6477731884700326, 0.11177869466966715, -0.6239979783236629, -0.16294755897685018])
        self.traj.addPostureWP([0.011620827893069757, 0.011620827893069757, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.21863093707070097, 0.053786022946714855, 0.009565941665818607, 1.5913111176938077, 0.11059585801161911, -0.84679789487427, -0.15214466815679062])
        self.traj.addPostureWP([0.010612848482630161, 0.010612848482630161, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.12376412584968774, 0.04708823679608376, 0.09863029599468474, 1.7746007143465548, 0.1626680243972309, 0.15723295611789795, -0.28198457588567716])
        self.traj.addPostureWP([0.010896271787838007, 0.010896271787838007, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.12925088686379427, 0.04784376129835639, 0.17075491784959057, 1.3765958292530334, 0.06815963504366353, 0.16164001735007272, -0.27280028203099144])
        self.traj.addPostureWP([0.011079324449831058, 0.011079324449831058, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.16703765767710585, 0.02674667580755949, 0.1664335578457814, 0.9426709269756101, 0.06474116939050259, 0.14929176280237405, -0.2858247746249341])
        self.traj.addPostureWP([0.010645399031272328, 0.010645399031272328, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.21262698415725878, 0.03189069453753046, 0.1509735711989367, 0.616687380054559, 0.06558743789004771, 0.16219172265488005, -0.2737297074742824])
        self.traj.addPostureWP([0.010885228709509176, 0.010885228709509176, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            -0.1441327086061609, 0.030221226869115037, 0.14990226397478432, 0.35741667678932476, 0.06851668939343862, 0.23867119549425891, -0.18921285088815107])

    def createFSM(self):
        # define the states
        execTrajState = TrajectoryState(self.dreamerInterface, self.traj)

        # wire the states into a FSM
        self.fsm = smach.StateMachine(outcomes=['exit'])
        with self.fsm:
            smach.StateMachine.add("RunTestTrajectory", execTrajState, 
                transitions={'done':'exit',
                             'exit':'exit'})

    def run(self):
        """
        Runs demo 12 behavior.
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

        print "Demo 12 done, waiting until ctrl+c is hit..."
        rospy.spin()  # just to prevent this node from exiting
        sis.stop()


# Main method
if __name__ == "__main__":
    rospy.init_node('Demo12_Test_6DOF_Traj', anonymous=True)
    demo = Demo12_Test6DOFTraj()
    demo.run()

    
