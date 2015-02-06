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
AMPLITUDE = 0.475
OFFSET = -0.215
FREQUENCY = 0.2

class NeckJointTest:
    def __init__(self):
        """
        The constructor.
        """

        # Define the dimensions of the message
        # dim = MultiArrayDimension()
        # dim.size = numDoFs
        # dim.label = "goalMsg"
        # dim.stride = 1

        # Define the goal messages
        self.goalMsgJ0 = Float64()

    def getSineSignal(self, elapsedTime_sec, amplitude, offset, freq_hz):
        return amplitude * math.sin(elapsedTime_sec * 2 * math.pi * freq_hz) + offset

    def start(self):
        """
        Publishes a sine wave trajectory for the lower neck extensor.
        Joint range of motion: 
          * Degrees: -40 to 15
          * Radians: -0.69 to 0.26
        """

        publisherJ0 = rospy.Publisher("/dreamer_controller/controlit/head/lower_neck_pitch/position_cmd", Float64, queue_size=1)
        startTime = time.time()

        while not rospy.is_shutdown():
            self.goalMsgJ0.data = self.getSineSignal(time.time() - startTime, AMPLITUDE, OFFSET, FREQUENCY)

            # Make the joint at jointIndex move in a sine wave.
            # Set all other joints to be at position zero.
            # for ii in range(0, numDoFs):
            #     self.goalMsg.data[ii] = 0
            # self.goalMsg.data[self.jointIndex] = goal

            publisherJ0.publish(self.goalMsgJ0)
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
    tester.start()
    # t = threading.Thread(target=neckJointTest.start)
    # t.start()

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
