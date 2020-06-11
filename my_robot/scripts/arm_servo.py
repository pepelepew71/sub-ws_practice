#!/usr/bin/env python

"""
Arm controller node for putting down and up of the sound-collector
"""

from __future__ import print_function
from __future__ import division

import rospy
from std_msgs.msg import Int16
from std_srvs.srv import Empty, EmptyResponse

def cb_putdown(request):
    PUB_S1.publish(S1_LB)
    PUB_S2.publish(S2_LB)
    return EmptyResponse()

def cb_pickup(request):
    PUB_S1.publish(S1_UB)
    PUB_S2.publish(S2_UB)
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='arm_servo', anonymous=False)

    # -- Get parameters
    s1_topic_name = rospy.get_param(param_name="~servo1_topic")
    s2_topic_name = rospy.get_param(param_name="~servo2_topic")

    S1_UB = rospy.get_param(param_name="~servo1_ub")
    S1_LB = rospy.get_param(param_name="~servo1_lb")
    S2_UB = rospy.get_param(param_name="~servo2_ub")
    S2_LB = rospy.get_param(param_name="~servo2_lb")

    # -- Node function
    rospy.Service(name='~putdown', service_class=Empty, handler=cb_putdown)
    rospy.Service(name='~pickup', service_class=Empty, handler=cb_pickup)

    PUB_S1 = rospy.Publisher(name=s1_topic_name, data_class=Int16, queue_size=5)
    PUB_S2 = rospy.Publisher(name=s2_topic_name, data_class=Int16, queue_size=5)

    rospy.spin()
