#!/usr/bin/env python3

import rospy
from threading import Thread
from math import atan, atan2, sqrt
from geometry_msgs.msg import Twist
from hexapodC import HexapodC
from movefunc import MoveFunc
from numpy import append, zeros
import time

jleg = ['Th1','Th2','Th3']
leg = ['1','2','3','4','5','6']
joints = []
for j in jleg:
    for l in leg:
        z = j + '_' + l
        joints.append(z)

class Planner:
    def __init__(self,robot, BH = 140.0, Ss = 100.0, Sh = 50.0, Rd = 283.71, p=0.0, r=0.0):
        self.di = 0.0
        self.BH = BH; self.Ss = Ss; self.Sh = Sh; self.Rd = Rd; self.p = p; self.r = r
        self.walk_vel = 0.0
        self.robot = robot
        self.Move = MoveFunc()

        self.pathsize = 7

        prevtime = time.time()*1000
        while not rospy.is_shutdown():
            currenttime = time.time()*1000
            self.Move.stand(self.BH,1000/3)
            self._set_angles(self.Move.theta1, self.Move.theta2, self.Move.theta3)
            if currenttime - prevtime >= 1000/3:
                ti = currenttime - prevtime 
                break
            rospy.sleep(0.01)

        self._th_walk = None
        self.running = False
        self.walking = False

        self._sub_cmd_vel = rospy.Subscriber(robot.ns + "cmd_vel", Twist, self._cb_cmd_vel, queue_size=1)

    def _cb_cmd_vel(self, msg):
        """Catches cmd_vel and update"""
        print ('cmdvel', msg)
        vx = msg.linear.x
        vy = msg.linear.y
        vt = msg.angular.z
        #self.set_velocity(vx, vy, vt)
        self.di = atan2(vy,vx)
        self.walk_vel = sqrt(vx**2+vy**2)
        if self.walk_vel != 0:
            tf =  self.Ss/(self.walk_vel*1000) # werk tf uit
            self.dt = tf/(self.pathsize-1)
            (self.x,self.y,self.z,self.xd,self.yd,self.zd) = self.Move.makepath_walk(self.Ss,self.Sh,self.Rd,self.p,self.r,self.di,self.BH,tf,self.pathsize)
        else:
            self.dt = 0.0
            self.x = zeros((6,self.pathsize*2-2)); self.y = zeros((6,self.pathsize*2-2)); self.z = zeros((6,self.pathsize*2-2))
            self.xd = zeros((6,self.pathsize*2-2)); self.yd = zeros((6,self.pathsize*2-2)); self.zd = zeros((6,self.pathsize*2-2))

        self.start()

    def _set_angles(self,th1,th2,th3):
        Angle = append(th1,[th2,th3])
        Angle = dict(zip(joints,Angle))
        self.robot.set_angles(Angle)
        rospy.loginfo(Angle)

    def start(self):
        if not self.running:
            self.running = True
            self._th_walk = Thread(target=self.do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self):
        if self.running:
            self.walking = False
            rospy.loginfo('Waiting for stopped')
            while not rospy.is_shutdown() and self._th_walk is not None:
                rospy.sleep(0.1)
            rospy.loginfo('Stopped')
            self.running = False
            
    def do_walk(self):
        """Main Walking Loop"""
        rospy.loginfo('Started walking thread')
        self.Move.prevtime = time.time()*1000

        while (not rospy.is_shutdown() and self.walking):
            print('do_walk')
            self.Move.setNextPathPoint(self.x,self.y,self.z,self.xd,self.yd,self.zd,self.dt*1000)
            self._set_angles(self.Move.theta1, self.Move.theta2, self.Move.theta3)
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('planner')
    rospy.sleep(1)

    rospy.loginfo('Instantiating hexapod Client')
    robot = HexapodC()
    rospy.loginfo('Instantiating hexapod Planner')
    walker = Planner(robot)

    rospy.loginfo('Planner Ready')
    while not rospy.is_shutdown():
        print('main')
        rospy.sleep(1)
