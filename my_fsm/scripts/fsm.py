#!/usr/bin/env python

from __future__ import print_function
from __future__ import division

import actionlib
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction
import rospy
from std_srvs.srv import Empty, EmptyResponse
import smach
import smach_ros

from my_states import PlanningState, NavState, BehaviorState

def cb_start(request):
    SM.execute()  # start FSM
    return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node(name='fsm', anonymous=False)

    #-- Get parameters
    ns_action = rospy.get_param(param_name="~ns_action")
    path_file = rospy.get_param(param_name="~path_file")
    top_cmd_vel = rospy.get_param(param_name="~top_cmd_vel")

    # -- Get service
    action_move_base = actionlib.SimpleActionClient(ns=ns_action, ActionSpec=MoveBaseAction)
    action_move_base.wait_for_server()

    # -- Get publisher
    pub_cmd_vel = rospy.Publisher(name=top_cmd_vel, data_class=Twist, queue_size=1)

    # -- Node function
    rospy.Service(name="~start", service_class=Empty, handler=cb_start)

    # -- FSM Config
    SM = smach.StateMachine(outcomes=['completed'])
    with SM:
        smach.StateMachine.add(
            label='PlanningState',
            state=PlanningState(path_file=path_file),
            transitions={'go': 'NavState', 'done': 'completed'})
        smach.StateMachine.add(
            label='NavState',
            state=NavState(action_move_base=action_move_base),
            transitions={'done': 'BehaviorState', 'failed': "PlanningState"})
        smach.StateMachine.add(
            label='BehaviorState',
            state=BehaviorState(pub_cmd_vel=pub_cmd_vel),
            transitions={'done': 'PlanningState'})

    # -- FSM VIEWER
    sis = smach_ros.IntrospectionServer('smach_server', SM, '/ROOT')
    sis.start()
    rospy.spin()
    sis.stop()
