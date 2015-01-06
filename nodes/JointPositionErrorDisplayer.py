#!/usr/bin/env python
import time
# import string

import rospy
from std_msgs.msg import Float64MultiArray
from controlit_core.srv import get_parameters

JOINT_INDICES_SERVICE = "/dreamer_controller/diagnostics/getRealJointIndices"

jointErrorMessage = None

def jointErrorCallback(data):
    global jointErrorMessage
    # message = "{0}: I heard: {1}".format(rospy.get_caller_id(), data)
    # rospy.loginfo(message)
    jointErrorMessage = data


if __name__ == '__main__':
    rospy.init_node('JointPositionErrorPrinter', anonymous=True)

    subscriber = rospy.Subscriber("/dreamer_controller/JPosTask/error", Float64MultiArray, jointErrorCallback)

    # Wait for the current Cartesian position and orientation measurements to arrive
    pauseCount = 0
    printWarning = False
    while not rospy.is_shutdown() and jointErrorMessage == None:
        time.sleep(0.5)
        pauseCount = pauseCount + 1
        if pauseCount > 5 and not printWarning:
            print "Waiting for joint error information..."
            printWarning = True

    subscriber.unregister()

    # Call the service that returns the joint indices
    print "Waiting for service {0}...".format(JOINT_INDICES_SERVICE)
    rospy.wait_for_service(JOINT_INDICES_SERVICE)
    try:
        print "Calling service {0}...".format(JOINT_INDICES_SERVICE)
        get_joint_indices_service = rospy.ServiceProxy(JOINT_INDICES_SERVICE, get_parameters)
        joint_indices_msg = get_joint_indices_service()
        # message = "Received joint indices: {0}".format(joint_indices_msg)
        # rospy.loginfo(message)

        resultList = [["Index", "Joint Name", "Error (rad)", "Error (deg)"]]
        resultList.append(["-----", "----------", "-----------", "-----------"])
        for joint in joint_indices_msg.params.values:
            # rospy.loginfo("{0} {1} {2}".format(joint.key, joint.value, jointErrorMessage.data[int(joint.key)]))
            errorRad = jointErrorMessage.data[int(joint.key)]
            errorDeg = errorRad / 3.14 * 180
            entry = [joint.key, joint.value, str(errorRad), str(errorDeg)]
            resultList.append(entry)
        
        col_width = max(len(word) for row in resultList for word in row) + 2  # padding
        resultTable = "\n"
        for row in resultList:
            resultTable = resultTable + "".join(word.ljust(col_width) for word in row)
            resultTable = resultTable + "\n"
        rospy.loginfo("Results:\n{0}".format(resultTable))

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e)

