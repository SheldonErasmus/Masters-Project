#!/usr/bin/env python3

import rospy
from threading import Thread
from math import atan, atan2, sqrt
from my_message.msg import PathVar_n_cmdVel
from hexapodC import HexapodC
from movefunc import MoveFunc
from numpy import append, zeros, ones
import time

class Planner:
    def __init__(self,robot, BH = 140.0, Ss = 100.0, Sh = 50.0, Rd = 283.71, p=0.0, r=0.0):
        self.di = 0.0
        self.BH = BH; self.Ss = Ss; self.Sh = Sh; self.Rd = Rd; self.p = p; self.r = r
        self.walk_vel = 0.0
        self.robot = robot
        self.Move = MoveFunc()
        
        self.pathsize = 7
        self.ang = zeros((1,self.pathsize*2-1))

        self._th_walk = None
        self.running = False
        self.walking = False

        self._sub_cmd_vel_path_var = rospy.Subscriber(robot.ns + "cmd_vel_path_var", PathVar_n_cmdVel, self._cb_cmdvel_pathVar, queue_size=1)


    def _cb_cmdvel_pathVar(self, msg):
        """Catches cmd_vel and path variables and update"""
        print ('cmdvel', msg)
        vx = msg.linear.x
        vy = msg.linear.y
        self.yaw = msg.angular.z

        self.BH = msg.path_var.BH
        self.Ss = msg.path_var.Ss
        self.Sh = msg.path_var.Sh
        self.Rd = msg.path_var.Rd
        self.p = msg.path_var.p
        self.r = msg.path_var.r

        self.di = atan2(vy,vx)
        self.walk_vel = sqrt(vx**2+vy**2)
        if self.walk_vel != 0:
            tf =  self.Ss/(self.walk_vel*1000) # werk tf uit
            self.dt = tf/(self.pathsize-1)
            (self.x,self.y,self.z,self.xd,self.yd,self.zd) = self.Move.makepath_walk(self.Ss,self.Sh,self.Rd,self.p,self.r,self.di,self.BH,tf,self.pathsize)
            (self.ang,z,self.ang_d,z_d) = self.Move.makepath_turn(self.yaw,0,tf,self.pathsize)
        elif self.walk_vel == 0 and self.yaw != 0:
            tf = 2
            self.dt = tf/(self.pathsize-1)
            (self.x,self.y,self.z,self.xd,self.yd,self.zd) = self.Move.makepath_walk(0,self.Sh,self.Rd,self.p,self.r,self.di,self.BH,tf,self.pathsize)
            (self.ang,z,self.ang_d,z_d) = self.Move.makepath_turn(self.yaw,0,tf,self.pathsize)
        else:
            tf = 2
            self.dt = -1
            (self.x,self.y,self.z,self.xd,self.yd,self.zd) = self.Move.makepath_walk(0,self.Sh,self.Rd,self.p,self.r,self.di,self.BH,tf,self.pathsize)
            self.z = ones((6,7*2-2))*-self.BH
            (self.ang,z,self.ang_d,z_d) = self.Move.makepath_turn(0,0,tf,self.pathsize)

        self.robot.sendpathflag = 1
        self.start()

    def start(self):
        if not self.running:
            self.running = True
            self._th_walk = Thread(target=self.do_walk)
            self._th_walk.start()
            self.walking = True

    def stop(self): #does not do anything yet
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
        pretime = time.time()*1000

        while (not rospy.is_shutdown() and self.walking):
            curtime = time.time()*1000
            if curtime - pretime >= 10:
                pretime = curtime
                print('do_walk')
                self.robot.set_path(self.x,self.y,self.z,self.ang,self.xd,self.yd,self.zd,self.ang_d,self.dt*1000)
            #rospy.sleep(0.01)

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
