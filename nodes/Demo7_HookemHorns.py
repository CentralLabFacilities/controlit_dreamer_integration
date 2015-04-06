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

class Demo7_HookemHorns:
    def __init__(self, dreamerInterface):
        self.dreamerInterface = dreamerInterface
        self.createTrajectories()

    def createTrajectories(self):

        # ==============================================================================================
        # Define the GoToReady trajectory
        self.trajHookEmHorns = Trajectory.Trajectory("HookEmHorns", 30.0) # 45.0

        # These are the initial values as specified in the YAML ControlIt! configuration file
        self.trajHookEmHorns.setInitRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])
        self.trajHookEmHorns.setInitLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajHookEmHorns.setInitRHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.setInitLHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.setInitPostureWP(DEFAULT_POSTURE)
        
        self.trajHookEmHorns.addRHCartWP([0.1538421760021268, -0.18969240926169845, 0.8079574432397748])
        self.trajHookEmHorns.addRHCartWP([0.2679889232274923, -0.1840815782703698, 0.9402466061369074])
        self.trajHookEmHorns.addRHCartWP([0.35675290561551704, -0.1896977676313571, 1.1526249884869435])
        self.trajHookEmHorns.addRHCartWP([0.3968625161007249, -0.2118354515626805, 1.367564444718767])
        self.trajHookEmHorns.addRHCartWP([0.3426800335832476, -0.22767093613877892, 1.6296487068409742])
        self.trajHookEmHorns.addRHCartWP([0.304275962172864, -0.2382959162647132, 1.749454177540091])
        self.trajHookEmHorns.addRHCartWP([0.30197107423870834, -0.3928828929486433, 1.6992185745614954])
        self.trajHookEmHorns.addRHCartWP([0.31326813417812516, -0.43939506162678943, 1.6599321424212767])
        self.trajHookEmHorns.addRHCartWP([0.32414228590604127, -0.34910413573587684, 1.6797489897827975])
        self.trajHookEmHorns.addRHCartWP([0.3220785651096926, -0.24395252173108958, 1.6812496838524162])
        self.trajHookEmHorns.addRHCartWP([0.43513522285735995, -0.256005663058001, 1.5603990676723727])
        self.trajHookEmHorns.addRHCartWP([0.47950569449579133, -0.26386621419287026, 1.4279755187621448])
        self.trajHookEmHorns.addRHCartWP([0.48321860539358535, -0.26331786592138423, 1.2778853575540223])
        self.trajHookEmHorns.addRHCartWP([0.4558300539540847, -0.26220512473380786, 1.164567999673142])
        self.trajHookEmHorns.addRHCartWP([0.4039109411551312, -0.24808232791560425, 1.0558342258837252])
        self.trajHookEmHorns.addRHCartWP([0.3334223664960938, -0.22768356581336102, 0.9520136435318634])
        self.trajHookEmHorns.addRHCartWP([0.22457608787770017, -0.22995326031396174, 0.8494531252948857])
        self.trajHookEmHorns.addRHCartWP([0.11349086677223105, -0.23155749901769876, 0.8021876802700734])
        self.trajHookEmHorns.addRHCartWP([0.033912978219317776, -0.29726881641499886, 0.82])

        self.trajHookEmHorns.addRHOrientWP([0.9376377012987288, 0.08391666275788233, 0.3373329732101732])
        self.trajHookEmHorns.addRHOrientWP([0.5298076308741426, 0.059524856512608346, 0.8460263977705906])
        self.trajHookEmHorns.addRHOrientWP([-0.3152818041068105, -0.019585535512386423, 0.9487959690037938])
        self.trajHookEmHorns.addRHOrientWP([-0.8042860069982763, 0.031037400974694453, 0.5934312923056084])
        self.trajHookEmHorns.addRHOrientWP([-0.9753115824035968, 0.05594906977279092, 0.21362822571221962])
        self.trajHookEmHorns.addRHOrientWP([-0.9551977674780248, 0.12583018234823193, 0.2678880180508416])
        self.trajHookEmHorns.addRHOrientWP([-0.953434882985859, 0.037368909210564676, 0.29927493802547905])
        self.trajHookEmHorns.addRHOrientWP([-0.9422648614304032, -0.0028969441860746997, 0.33485599685226736])
        self.trajHookEmHorns.addRHOrientWP([-0.9242880039874043, 0.17665085500131472, 0.338358036866696])
        self.trajHookEmHorns.addRHOrientWP([-0.9521416802577293, 0.19929752012556518, 0.23174710178940408])
        self.trajHookEmHorns.addRHOrientWP([-0.7838347773806876, 0.1704775043563853, 0.597110092258527])
        self.trajHookEmHorns.addRHOrientWP([-0.5858893225897619, 0.05207435199741848, 0.8087162441421336])
        self.trajHookEmHorns.addRHOrientWP([-0.3195867517716314, 0.05716145080116825, 0.9458313151055864])
        self.trajHookEmHorns.addRHOrientWP([-0.08153226677973102, 0.0485513174389581, 0.9954874479613995])
        self.trajHookEmHorns.addRHOrientWP([0.18058148815364847, -0.02854626048843662, 0.983145684600375])
        self.trajHookEmHorns.addRHOrientWP([0.42334613103604013, -0.08713471946197092, 0.9017680378019058])
        self.trajHookEmHorns.addRHOrientWP([0.7697346850293526, -0.11596858259793173, 0.6277418279141436])
        self.trajHookEmHorns.addRHOrientWP([0.9329380163771549, -0.12167477217931731, 0.33885381422284494])
        self.trajHookEmHorns.addRHOrientWP([1.0, 0.0, 0.0])

        self.trajHookEmHorns.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajHookEmHorns.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajHookEmHorns.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajHookEmHorns.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])
        self.trajHookEmHorns.addLHCartWP([0.033912978219317776, 0.29726881641499886, 0.82])

        self.trajHookEmHorns.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.addLHOrientWP([1.0, 0.0, 0.0])
        self.trajHookEmHorns.addLHOrientWP([1.0, 0.0, 0.0])

        self.trajHookEmHorns.addPostureWP([0.02765420368352817, 0.02765420368352817, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.13662820530020137, -0.039976323893372405, -0.05378020503784494, 0.32414594307518363, -0.04263777616613594, -0.06458534659603916, -0.005402889639751004])
        self.trajHookEmHorns.addPostureWP([0.027348077714022364, 0.027348077714022364, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.04547044262792076, -0.04365906508126285, -0.05855716880573466, 1.126498131343053, -0.06624447187653187, -0.11178278707876864, -0.0014990398066508197])
        self.trajHookEmHorns.addPostureWP([0.0274237005867097, 0.0274237005867097, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.3089672980423415, -0.04445379807455004, -0.052814135487953204, 1.611723186391318, -0.02799101829201779, 0.024361654748688056, -0.08093840742015859])
        self.trajHookEmHorns.addPostureWP([0.028397399986105746, 0.028397399986105746, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.8653996071703635, -0.017109373941721375, -0.0007408119630878939, 1.653758594343431, -0.048038821228816024, 0.043825319849869475, -0.032401921005653096])
        self.trajHookEmHorns.addPostureWP([0.028085499678190427, 0.028085499678190427, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.6656145751406148, -0.005218975413146829, 0.04710737418157687, 1.3613511108833538, -0.07276986049943275, -0.041697737663978045, -0.02789645531009445])
        self.trajHookEmHorns.addPostureWP([0.027971502785403425, 0.027971502785403425, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            2.11699440277832, 0.024606851851254226, 0.04862200357685229, 0.9178597218907055, -0.14303191457592024, -0.10329084631536531, 0.03870436729705735])
        self.trajHookEmHorns.addPostureWP([0.02804342261091174, 0.02804342261091174, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            2.0210290469832595, 0.2824994210990194, 0.24978431344145177, 0.936682417640926, 0.013056538292092277, -0.09549847313258142, 0.04081191611695253])
        self.trajHookEmHorns.addPostureWP([0.027980406666572685, 0.027980406666572685, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.939708623941373, 0.3313251034922685, 0.3827002729416129, 0.939604516129285, 0.015089605878083513, -0.09177237830403642, 0.037951268876058424])
        self.trajHookEmHorns.addPostureWP([0.0274827368860405, 0.0274827368860405, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.895762704245951, 0.28531165082706517, 0.07021236235326105, 1.046852475752334, 0.015562036918602227, -0.08890700812865812, 0.04514001900000088])
        self.trajHookEmHorns.addPostureWP([0.027305247445866994, 0.027305247445866994, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.8570277563473854, 0.15863940551121433, -0.12377770179714928, 1.1959859436494313, -0.0035478233822314295, -0.08953904659349636, 0.045257490814717985])
        self.trajHookEmHorns.addPostureWP([0.028533650065815493, 0.028533650065815493, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.5540056388327872, 0.15984203324564575, -0.09989609154992295, 1.0906899806637205, 0.016996048070825963, -0.0887940094511041, 0.0618471595027266])
        self.trajHookEmHorns.addPostureWP([0.02860208047521242, 0.02860208047521242, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            1.2502399921060512, 0.1592188803644951, -0.06555710543142508, 1.0977652063999188, 0.1180287330664312, -0.08849317041459447, 0.058677809623456006])
        self.trajHookEmHorns.addPostureWP([0.028426031311457156, 0.028426031311457156, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.9286168410427282, 0.1574762729738452, -0.06469744888513598, 1.09912842949099, 0.11147539133293521, -0.06961905360776385, 0.07498215882506887])
        self.trajHookEmHorns.addPostureWP([0.02890890836509012, 0.02890890836509012, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.6886649033652121, 0.15169144780174829, -0.0625923994167016, 1.0859306967983935, 0.11362018220307567, -0.059920278031326234, 0.08767530614359927])
        self.trajHookEmHorns.addPostureWP([0.028757579265564104, 0.028757579265564104, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.45629726472147286, 0.0944864874949844, -0.030569368267476825, 1.0496396161594679, 0.1243527769664117, -0.059344560236435695, 0.09309371709642968])
        self.trajHookEmHorns.addPostureWP([0.02822536896851773, 0.02822536896851773, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.27770196066699576, 0.03195094740485626, -0.01378350357594359, 0.9056579035486473, 0.12090147918050226, 0.004333492577621253, 0.0029023206116514984])
        self.trajHookEmHorns.addPostureWP([0.027687559848942545, 0.027687559848942545, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.15825724179565157, 0.03048317382365852, -0.01289243378319844, 0.5763323474183744, 0.14372268400157334, 0.0026840504424737646, 0.004432612213345542])
        self.trajHookEmHorns.addPostureWP([0.02773148095605579, 0.02773148095605579, 
            0.0, 0.174532925, 0.0, 0.174532925, 0.0, 0.0, 0.0,  # left arm
            0.04838444327506801, 0.030814623201507046, -0.014466793019855949, 0.351297189049863, 0.14617716590017665, 0.001240016318019907, 0.004207804245329691])
        self.trajHookEmHorns.addPostureWP(DEFAULT_POSTURE)
        
    def run(self, enablePrompts = True):
        """
        Runs the Cartesian and orientation demo 7 behavior.
        """

        if not self.dreamerInterface.connectToControlIt(DEFAULT_POSTURE):
            return

        # print "Trajectory Wave:\n {0}".format(self.trajShake)

        if enablePrompts:
            response = raw_input("Start demo? Y/n\n")
            if response == "N" or response == "n":
                return

        self.dreamerInterface.closeRightHand(includePinky = False, includeMiddle = True, includeIndex = False)

        #=============================================================================
        if not self.dreamerInterface.followTrajectory(self.trajHookEmHorns):
            return

        print "Opening right hand."
        self.dreamerInterface.openRightHand()

# Main method
if __name__ == "__main__":

    rospy.init_node('Demo7_HookemHorns')

    dreamerInterface = DreamerInterface.DreamerInterface(ENABLE_USER_PROMPTS)

    demo = Demo7_HookemHorns()
    demo.run()

    print "Demo 7 done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting
