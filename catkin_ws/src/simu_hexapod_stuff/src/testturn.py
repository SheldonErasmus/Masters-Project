#!/usr/bin/env python3

import rospy
from hexapodC import HexapodC

if __name__ == '__main__':
    rospy.init_node("turn")
    robot = HexapodC()
    rospy.sleep(1)

    robot.set_walk_velocity(0.0,0,15)
    angle = 0
    while angle < 90:
        angle = angle + 15
        print(angle)
        rospy.sleep(2)

    robot.set_walk_velocity(0.0,0,0)