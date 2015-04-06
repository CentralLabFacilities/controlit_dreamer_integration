#!/usr/bin/env python

'''
Make the neck joints move. Useful for testing and debugging purposes.
'''

import sys, getopt     # for getting and parsing command line arguments
import time
import math
import threading
import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension

UPDATE_PERIOD = 0.01  # 100Hz

def degToRad(degValue):
    return degValue / 180.0 * 3.14159265359

class NeckJointDetails:
    def __init__(self, name, index, minPos, maxPos, freq = 0.02):
        self.name = name
        self.index = index
        self.minPos = minPos
        self.maxPos = maxPos
        self.freq = freq

    def getAmplitude(self):
        return degToRad(self.maxPos - self.minPos) / 2.0

    def getOffset(self):
        return degToRad(self.minPos) + self.getAmplitude()

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "[name = {0}, index = {1}, minPos = {2}, maxPos = {3}, freq = {4}]".format(
            self.name, self.index, self.minPos, self.maxPos, self.freq)

class NeckJointTest:
    def __init__(self):
        """
        The constructor.
        """

        # Define the joint info
        self.jointSpecs = []
        self.jointSpecs.append(NeckJointDetails("lower_neck_pitch", 0, -40, 15))
        self.jointSpecs.append(NeckJointDetails("upper_neck_yaw", 1, -80, 80))
        self.jointSpecs.append(NeckJointDetails("upper_neck_roll", 2, -14, 14))
        self.jointSpecs.append(NeckJointDetails("upper_neck_pitch", 3, -7, 37))
        self.jointSpecs.append(NeckJointDetails("eye_pitch", 4, -34, 34))
        self.jointSpecs.append(NeckJointDetails("right_eye_yaw", 5, -34, 34))
        self.jointSpecs.append(NeckJointDetails("left_eye_yaw", 6, -34, 34))

        # Define the goal message
        # Create the goal message. Store the current joint states into the messages
        dimPos = MultiArrayDimension()
        dimPos.size = len(self.jointSpecs)
        dimPos.label = "goalPosMsg"
        dimPos.stride = 1

        self.goalPosMsg = Float64MultiArray()
        for ii in range(0, len(self.jointSpecs)):
            self.goalPosMsg.data.append(0)
        self.goalPosMsg.layout.dim.append(dimPos)
        self.goalPosMsg.layout.data_offset = 0

        # dimVel = MultiArrayDimension()
        # dimVel.size = len(self.jointSpecs)
        # dimVel.label = "goalVelMsg"
        # dimVel.stride = 1

        # self.goalVelMsg = Float64MultiArray()
        # for ii in range(0, len(self.jointSpecs)):
        #     self.goalVelMsg.data.append(0)
        # self.goalVelMsg.layout.dim.append(dimVel)
        # self.goalVelMsg.layout.data_offset = 0


        self.cmdPublisher = rospy.Publisher("/dreamer_controller/controlit/head/position_cmd", Float64MultiArray, queue_size=1)

    def getUserOptions(self):
        question = "Which head joint would you like to test? (or type 'q' to exit)\n"
        for ii in range(0, len(self.jointSpecs)):
            question = question + "  {0}: {1}\n".format(ii, self.jointSpecs[ii].name)

        question = question + "$ "

        index = raw_input(question)
        
        if "q" == index:
           return False

        self.jointIndex = int(index)

        defaultFreq = self.jointSpecs[self.jointIndex].freq

        index = raw_input("Please enter desired frequency (default is {0}): ".format(defaultFreq))

        if "q" == index:
           return False

        if not index == "":
            self.jointSpecs[self.jointIndex].freq = float(index)

        return True

    def getSineSignal(self, elapsedTime_sec, amplitude, offset, freq_hz):
        return amplitude * math.sin(elapsedTime_sec * 2 * math.pi * freq_hz) + offset

    def stop(self):
        self.keepRunning = False

    def start(self):
        """
        Publishes a sine wave trajectory for the specified joint.
        """

        self.keepRunning = True

        startTime = time.time()

        name      = self.jointSpecs[self.jointIndex].name
        amplitude = self.jointSpecs[self.jointIndex].getAmplitude()
        offset    = self.jointSpecs[self.jointIndex].getOffset()
        freq      = self.jointSpecs[self.jointIndex].freq

        print "Publishing sine wave to joint {0}:\n"\
              "  - frequency: {1}\n"\
              "  - offset: {2}\n"\
              "  - amplitude: {3}\n".format(name, freq, offset, amplitude)

        while not rospy.is_shutdown() and self.keepRunning:
            for ii in range(0, len(self.jointSpecs)):
                self.goalPosMsg.data[ii] = 0

            self.goalPosMsg.data[self.jointIndex] = self.getSineSignal(time.time() - startTime, amplitude, offset, freq)

            self.cmdPublisher.publish(self.goalPosMsg)
            rospy.sleep(UPDATE_PERIOD)

# Main method
if __name__ == "__main__":

    rospy.init_node('NeckJointTest', anonymous=True)

    # Define default values for the command line arguments
    # rosTopic = '/JPosTask/goalPosition'
    # amplitude = 1
    # offset = 0
    # initGoal = 0
    # numDoFs  = 1
    # jointIndex = 0
    # period   = 1.0
    # updateFreq = 50

    # usageStr = "Usage: python NeckJointTest.py [parameters]\n"\
    #            "Valid parameters include:\n"\
    #            " -t or --rosTopic [ros topic] (default {0})\n"\
    #            " -a or --amplitude [amplitude] (default {1})\n"\
    #            " -o or --offset [offset, the vertical offset of the sine waves] (default {2})\n"\
    #            " -i or --initGoal [init goal, the initial value of the sine waves] (default {3})\n"\
    #            " -n or --numDoFs [number of DoFs] (default {4})\n"\
    #            " -j or --jointIndex [joint index], the index of the joint to move in a sine wave (default {5}\n"\
    #            " -p or --period [period, the period of the sine wave] (default {6})\n"\
    #            " -f or --updateFreq [update frequency, the rate at which new points along the trajectory are published in Hz] (default {7})".format(
    #     rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq)

    # Parse the command line arguments
    # try:
    #     opts, args = getopt.getopt(sys.argv[1:],"ht:a:o:i:n:j:p:f:",["rosTopic=", "amplitude=", "offset=", "initGoal=", "numDoFs=", "jointIndex=", "period=", "updateFreq="])
    # except getopt.GetoptError:
    #    rospy.logerr(usageStr)
    #    sys.exit(2)

    # for opt, arg in opts:
    #     if opt == '-h':
    #         rospy.loginfo(usageStr)
    #         sys.exit()
    #     elif opt in ("-t", "--rosTopic"):
    #         rosTopic = arg
    #     elif opt in ("-a", "--amplitude"):
    #         amplitude = float(arg)
    #     elif opt in ("-o", "--offset"):
    #         offset = float(arg)
    #     elif opt in ("-i", "--initGoal"):
    #         initGoal = float(arg)
    #     elif opt in ("-n", "--numDoFs"):
    #         numDoFs = int(arg)
    #     elif opt in ("-j", "--jointIndex"):
    #         jointIndex = int(arg)            
    #     elif opt in ("-p", "--period"):
    #         period = float(arg)
    #     elif opt in ("-f", "--updateFreq"):
    #         updateFreq = float(arg)
    #     else:
    #         print "Unknown argument \"{0}\"".format(opt)

    # rospy.loginfo("Input parameters:\n"\
    #               "  - ROS topic: {0}\n"\
    #               "  - Amplitude: {1}\n"\
    #               "  - Offset: {2}\n"\
    #               "  - Init Goal: {3}\n"\
    #               "  - Number of DoFs: {4}\n"\
    #               "  - Joint Index: {5}\n"\
    #               "  - Period: {6}\n"\
    #               "  - Update Frequency: {7}".format(
    #     rosTopic, amplitude, offset, initGoal, numDoFs, jointIndex, period, updateFreq))


    # Create a NeckJointTest object
    tester = NeckJointTest()

    done = False
    while not rospy.is_shutdown() and not done:
        if tester.getUserOptions():
            thread = threading.Thread(target=tester.start)
            thread.start()

            index = raw_input("Type any key to return to main menu: ")
            tester.stop()
            thread.join()
        else:
            done = True

    # done = False
    # while not done:
    #     index = raw_input("Change sine wave to index (type q to exit): ")

    #     if "q" == index:
    #         done = True
    #     elif int(index) >= numDoFs:
    #         print "ERROR: Tried to set index to a value greater than the number of DOFs ({0})!".format(numDoFs)
    #     else:
    #         print "Setting joint index to be {0}".format(index)
    #         NeckJointTest.setJointIndex(int(index))

    rospy.signal_shutdown("done")
