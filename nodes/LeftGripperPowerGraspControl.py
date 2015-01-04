#!/usr/bin/env python

'''
Allows a user to control Dreamer's left gripper power grasp.
'''
import rospy
import threading
import RightHandPowerGraspControl

# Main method
if __name__ == "__main__":

    rospy.init_node('LeftHandPowerGraspControl', anonymous=True)

    # Define default values for the command line arguments
    rosTopic = '/dreamer_controller/controlit/leftGripper/powerGrasp'
    
    # Create a RightHandPowerGraspControl object
    leftHandPowerGraspControl = RightHandPowerGraspControl.RightHandPowerGraspControl(rosTopic)
    t = threading.Thread(target=leftHandPowerGraspControl.start)
    t.start()

    done = False
    while not done:
        index = raw_input("Left gripper power grasp state (type t, f, or q to exit): ")

        if "q" == index:
            done = True
        elif "t" == index:
            leftHandPowerGraspControl.doPowerGrasp()
        elif "f" == index:
            leftHandPowerGraspControl.relaxPowerGrasp()
        else:
            print "Unknown command {0}".format(index)

    rospy.signal_shutdown("done")
