#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import movefunc
from numpy import append

mf = movefunc.MoveFunc()
ns = '/simple_hexapod/'
rospy.init_node('planner')

joints = None
angle = None

rospy.loginfo('Hello')

def cb_joints(data):
    global joints, angles
    if joints is None:
        joints = data.name
    angles = data.position

def set_angles():
    Angle = append(mf.theta1,[mf.theta2,mf.theta3])
    Angle_dic = dict(zip(joints,Angle))
    #rospy.loginfo(Angle_dic)
    return Angle_dic

sub_joints = rospy.Subscriber(ns+'joint_states', JointState, cb_joints, queue_size=10)
while not rospy.is_shutdown():
    if joints is not None:
        break
    rospy.sleep(0.1)
    rospy.loginfo('waiting for joints')
rospy.loginfo('joints populated')

rospy.loginfo('Creating angle publishers')
pub_angles = {}
for j in joints:
    pub = rospy.Publisher(ns+j+'_position_controller/command',Float64,queue_size=1)
    pub_angles[j] = pub 

BH = 140.0; Ss = 100; Sh = 50; Rd = 283.71; p=0; r=0; di=0
mf.stand(BH)
Angle_dic = set_angles()
for j,v in Angle_dic.items():
    pub_angles[j].publish(v)

(xp,yp,zp,xp_d,yp_d,zp_d) = mf.makepath_walk(Ss, Sh, Rd, p, r, di, BH)


while not rospy.is_shutdown():
    mf.setNextPathPoint(xp,yp,zp,xp_d,yp_d,zp_d)
    Angle_dic = set_angles()
    for j,v in Angle_dic.items():
        pub_angles[j].publish(v)
    rospy.sleep(0.1)
 

