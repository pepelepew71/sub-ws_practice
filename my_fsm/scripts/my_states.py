#!/usr/bin/env python

from math import radians
import time

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
import rospy
import smach
from tf.transformations import quaternion_from_euler


class PlanningState(smach.State):

    def __init__(self, path_file):
        super(PlanningState, self).__init__(outcomes=('go', 'done'), output_keys=["target"])
        self._waypoints = parse_file(path_file=path_file)
        self._count = 0

    def execute(self, userdata):
        rospy.loginfo('fsm/PlanningState')
        userdata.target = None
        time.sleep(1)  # <<<
        if self._count < len(self._waypoints):
            userdata.target = self._waypoints[self._count]
            self._count += 1
            return 'go'
        else:
            self._count = 0
            return 'done'


class NavState(smach.State):

    def __init__(self, action_move_base):
        super(NavState, self).__init__(outcomes=['done', 'failed'], input_keys=["target"], output_keys=["behavior"])
        self.action_move_base = action_move_base
        self.status = None

    def execute(self, userdata):
        x_m, y_m, rz_deg, behavior = userdata.target
        userdata.behavior = behavior

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_m
        goal.target_pose.pose.position.y = y_m
        q = quaternion_from_euler(0.0, 0.0, radians(rz_deg))
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo('fsm/NavState : {} {} {}'.format(x_m, y_m, rz_deg))
        self.status = None
        self.action_move_base.send_goal(goal=goal, feedback_cb=None, done_cb=self.done_cb)
        self.action_move_base.wait_for_result()

        if self.status == 3:
            return 'done'
        else:
            return 'failed'

    def done_cb(self, status, result):
        # -- http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        self.status = status


class BehaviorState(smach.State):

    def __init__(self, pub_cmd_vel):
        super(BehaviorState, self).__init__(outcomes=['done'], input_keys=["behavior"])
        self.pub_cmd_vel = pub_cmd_vel

    def execute(self, userdata):
        if userdata.behavior == 1:
            rospy.loginfo("fsm/BehaviorState : rotating")
            self.rotate(wz=1.0, sec=2.0)
            self.rotate(wz=-1.0, sec=4.0)
            self.rotate(wz=1.0, sec=2.0)
        else:
            rospy.loginfo("fsm/BehaviorState : just a waypoint")
            time.sleep(1)
        return 'done'

    def rotate(self, wz, sec):
        rate = rospy.Rate(hz=5)
        t_start = rospy.get_rostime().to_sec()
        while True:
            twist = Twist()
            t_now = rospy.get_rostime().to_sec()
            if t_now - t_start < sec:
                twist.angular.z = wz
                self.pub_cmd_vel.publish(twist)
            else:
                self.pub_cmd_vel.publish(twist)  # zero cmd
                break
            rate.sleep()


def parse_file(path_file):

    with open(name=path_file, mode="r") as fileIO:
        _lines = fileIO.readlines()

    waypoints = list()
    for line in _lines:
        if line and not line.isspace():
            target = tuple(float(i) for i in line.split(","))  # (x_m, y_m, rz_deg, behavior)
            waypoints.append(target)

    return waypoints

if __name__ == "__main__":

    path_file = "../file/waypoints.csv"
    waypoints = parse_file(path_file=path_file)

    for p in waypoints:
        print(p)
