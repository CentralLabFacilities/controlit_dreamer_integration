#!/usr/bin/env python
import time

import rospy
from std_msgs.msg import Float64MultiArray
from controlit_core.srv import get_parameters

jointErrorMessage = 0

def jointErrorCallback(data):
    global jointErrorMessage
    # message = "{0}: I heard: {1}".format(rospy.get_caller_id(), data)
    # rospy.loginfo(message)
    jointErrorMessage = data


if __name__ == '__main__':
    rospy.init_node('JointPositionErrorPrinter', anonymous=True)

    subscriber = rospy.Subscriber("/dreamer_controller/JPosTask/error", Float64MultiArray, jointErrorCallback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rospy.loginfo("Waiting for joint error message to arrive...")
    while jointErrorMessage == 0:
        time.sleep(.01)

    message = "Received joint error message:\n{0}".format(jointErrorMessage)
    rospy.loginfo(message)

    subscriber.unregister()

    # Call the service that returns the joint indices
    rospy.wait_for_service('/dreamer_controller/diagnostics/getJointIndices')
    try:
        get_joint_indices_service = rospy.ServiceProxy('/dreamer_controller/diagnostics/getJointIndices', get_parameters)
        resp1 = get_joint_indices_service()
        message = "Received joint indices: {0}".format(resp1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

