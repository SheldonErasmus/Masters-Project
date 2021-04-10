#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

ns = '/simple_hexapod/'
#rospy.init_node('motor_controlers')

def pp():
    x = {}
    x['nana'] = 5
    print(x)
    vv(x)
    print(x)

def vv(x):
    x['nana'] = x['nana'] + x['nana']
    print(x)

pp()
