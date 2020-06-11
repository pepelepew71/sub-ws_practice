#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

from math import radians

import rospy
from std_msgs.msg import Int16, Float64

def cb_s1(response):
    PUB_S1.publish(radians(response.data))

def cb_s2(response):
    PUB_S2.publish(radians(response.data))

if __name__ == "__main__":

    rospy.init_node(name='arm_controller', anonymous=False)

    # -- Get parameters
    s1_topic_name = rospy.get_param(param_name="~servo1_topic")
    s2_topic_name = rospy.get_param(param_name="~servo2_topic")

    s1_ub = rospy.get_param(param_name="~servo1_ub")
    s1_lb = rospy.get_param(param_name="~servo1_lb")
    s2_ub = rospy.get_param(param_name="~servo2_ub")
    s2_lb = rospy.get_param(param_name="~servo2_lb")

    # -- Node function
    rospy.Subscriber(name=s1_topic_name, data_class=Int16, callback=cb_s1)
    rospy.Subscriber(name=s2_topic_name, data_class=Int16, callback=cb_s2)

    PUB_S1 = rospy.Publisher(name="/joint1_position_controller/command", data_class=Float64, queue_size=5)
    PUB_S2 = rospy.Publisher(name="/joint2_position_controller/command", data_class=Float64, queue_size=5)

    rospy.spin()
