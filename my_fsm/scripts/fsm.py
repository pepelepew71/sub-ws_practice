#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import rospy
import smach
import smach_ros
from std_srvs.srv import Empty, EmptyResponse

from ros_gps_nav.srv import GoalGPS, GoalGPSResponse
from my_state import PlanState, NavState, BehaviorState

def cb_start(request):
    SM.execute()
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    # -- Get parameter
    path_file = rospy.get_param(param_name="~path_file")
    service_name_goal = rospy.get_param(param_name="~service_goal")
    service_name_pickup = rospy.get_param(param_name="~service_pickup")
    service_name_putdown = rospy.get_param(param_name="~service_putdown")

    # -- Get service
    rospy.wait_for_service(service=service_name_goal)
    rospy.wait_for_service(service=service_name_pickup)
    rospy.wait_for_service(service=service_name_putdown)

    service_goal = rospy.ServiceProxy(name=service_name_goal, service_class=GoalGPS)
    service_pickup = rospy.ServiceProxy(name=service_name_pickup, service_class=Empty)
    service_putdown = rospy.ServiceProxy(name=service_name_putdown, service_class=Empty)

    # -- Node function
    rospy.Service(name="~start", service_class=Empty, handler=cb_start)

    # -- FSM
    SM = smach.StateMachine(outcomes=['completed'])

    with SM:
        smach.StateMachine.add(
            label='PlanState',
            state=PlanState(path_file=path_file),
            transitions={'go': 'NavState', 'done': 'completed'})
        smach.StateMachine.add(
            label='NavState',
            state=NavState(service_goal=service_goal),
            transitions={'done': 'BehaviorState', 'failed': "PlanState"})
        smach.StateMachine.add(
            label='BehaviorState',
            state=BehaviorState(service_pickup=service_pickup, service_putdown=service_putdown),
            transitions={'done': 'PlanState'})

    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    rospy.spin()
    sis.stop()
