#!/usr/bin/env python

import time

import rospy
import smach


class PlanState(smach.State):

    def __init__(self, path_file):
        super(PlanState, self).__init__(outcomes=('go', 'done'), output_keys=["target"])
        self._waypoints = parse_file(path_file=path_file)
        self._count = 0

    def execute(self, userdata):
        rospy.loginfo('fsm/ParserState')
        userdata.target = None
        time.sleep(1)
        if self._count < len(self._waypoints):
            userdata.target = self._waypoints[self._count]
            self._count += 1
            return 'go'
        else:
            self._count = 0
            return 'done'


class NavState(smach.State):

    def __init__(self, service_goal):
        super(NavState, self).__init__(outcomes=['done', 'failed'], input_keys=["target"], output_keys=["num"])
        self.service_goal = service_goal

    def execute(self, userdata):
        lat, lon, num = userdata.target
        rospy.loginfo('fsm/NavState : {} {}'.format(lat, lon))
        response = self.service_goal(lat, lon)
        rospy.loginfo('fsm/NavState state: ' + str(response.state))
        if response.state == 3:
            userdata.num = num
            return 'done'
        else:
            userdata.num = 0
            return 'failed'


class BehaviorState(smach.State):

    def __init__(self, service_pickup, service_putdown):
        super(BehaviorState, self).__init__(outcomes=['done'], input_keys=["num"])
        self.service_pickup = service_pickup
        self.service_putdown = service_putdown

    def execute(self, userdata):
        if userdata.num == 1:
            rospy.loginfo("fsm/BegaviorState : put down")
            self.service_putdown()
            time.sleep(5)
            rospy.loginfo("fsm/BegaviorState : pick up")
            self.service_pickup()
            time.sleep(1)
        else:
            pass
        return 'done'


def parse_file(path_file):

    with open(name=path_file, mode="r") as fileIO:
        data_raw = fileIO.readlines()

    data_nav = list()

    for line in data_raw:
        navs = tuple(float(i) for i in line.split())  # latitude, longtitude, num
        data_nav.append(navs)

    waypoints = list()
    waypoints.append(data_nav[0])

    for i in range(len(data_nav)-1):
        p1, p2 = data_nav[i:i+2]
        if p2[2] > 1:
            d_lat = (p2[0] - p1[0]) / p2[2]
            d_lon = (p2[1] - p1[1]) / p2[2]
            for d in range(1, int(p2[2]+1)):
                lat = p1[0] + d_lat*d
                lon = p1[1] + d_lon*d
                waypoints.append((lat, lon, 1))
        else:
            waypoints.append(p2)

    return waypoints

if __name__ == "__main__":

    path_file = "/home/ych/ws/src/ros_practice/my_fsm/files/nav.txt"
    waypoints = parse_file(path_file=path_file)

    for p in waypoints:
        print(p)
