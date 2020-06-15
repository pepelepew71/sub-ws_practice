#!/usr/bin/env python

import time

import rospy
import smach


class PlanState(smach.State):

    def __init__(self, param_name_nav_setup):
        super(PlanState, self).__init__(outcomes=('go', 'done'), output_keys=["target"])
        csv_txt = rospy.get_param(param_name=param_name_nav_setup)  # make sure /nav_setup has value
        self._waypoints = parse_csv(csv_txt=csv_txt)
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


def parse_csv(csv_txt):

    data_nav = list()
    lines = csv_txt.split("\n")

    for line in lines:
        if line and not line.isspace():
            navs = tuple(float(i) for i in line.split(","))  # latitude, longtitude, num
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

    csv_txt = """
    25.0135830014,121.547412085,1
    25.0135643779,121.547356729,2
    25.0135850055,121.547267552,2
    25.0135107002,121.547275854,2
    25.0134463139,121.547274833,2
    25.0134452747,121.547341385,2
    25.0134443431,121.547411426,2
    25.0135088570,121.547412937,2
    25.0135093206,121.547344050,1
    """

    waypoints = parse_csv(csv_txt=csv_txt)

    for p in waypoints:
        print(p)
