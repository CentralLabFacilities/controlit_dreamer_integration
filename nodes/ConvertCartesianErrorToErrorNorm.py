#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Float64MultiArray, MultiArrayDimension
import math


ERROR_TOPIC = "/dreamer_controller/RightHandPosition/error"
# ERROR_TOPIC = "/dreamer_controller/LeftHandPosition/error"

def callback(errorMsg):
    # print errorMsg
    errorNormMsg = Float64()
    errorNormMsg.data = math.sqrt(errorMsg.data[0] * errorMsg.data[0] + errorMsg.data[1] * errorMsg.data[1] + errorMsg.data[2] * errorMsg.data[2])
    # print "errorNorm = {0}".format(errorNormMsg.data)
    pub.publish(errorNormMsg)
    

rospy.init_node('node_name')

pub = rospy.Publisher(ERROR_TOPIC + "Norm", Float64, queue_size=10)

rospy.Subscriber(ERROR_TOPIC, Float64MultiArray, callback)
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()