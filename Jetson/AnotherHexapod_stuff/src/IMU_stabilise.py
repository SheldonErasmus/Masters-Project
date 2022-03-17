#!/usr/bin/env python3

import rospy
from hexapodC import HexapodC
from ToEulerAngles import ToEulerAng
import time
from marvelmind_nav.msg import hedge_imu_fusion
from geometry_msgs.msg import Vector3

class PIDcontroller:
    def __init__(self,Kp,Ki,Kd,tau,T):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd 
        self.tau = tau
        self.T = T

        self.i1 = 0.0
        self.d1 = 0.0
        self.e1 = 0.0
        self.u = 0.0

    def PIDUpdate(self,SetPoint,measurement):

        e0 = SetPoint - measurement

        p0 = self.Kp*e0

        i0 = self.i1 + 0.5*self.Ki*self.T*(e0+self.e1)

        d0 = (2*self.Kd*(e0-self.e1) + (2*self.tau-self.T)*self.d1)/(2*self.tau+self.T)

        self.u = p0 + i0 + d0

        self.i1 = i0
        self.d1 = d0
        self.e1 = e0

        return self.u
        

measure_roll = 0.0; measure_pitch = 0.0
ref_roll = 0.0; ref_pitch = 0.0
roll_input = 0.0; pitch_input = 0.0

def imu_cb(msg):
    global measure_roll,measure_pitch,roll_input,pitch_input
    (measure_roll,measure_pitch,yaw) = ToEulerAng(msg.qx,msg.qy,msg.qz,msg.qw)

    roll_input = rollController.PIDUpdate(ref_roll,measure_roll)
    pitch_input = pitchController.PIDUpdate(ref_pitch,measure_pitch)

    pubRollPitch.publish(x = pitch_input,y=roll_input)

    #print(measure_roll,measure_pitch)
    print(roll_input)

if __name__ == '__main__':

    rollController = PIDcontroller(1.1,2.21,0.032,0.02,100)
    pitchController = PIDcontroller(1.1,2.21,0.032,0.02,100)

    rospy.init_node("IMU_stabilise")
    pubRollPitch = rospy.Publisher("RollPitch_input",Vector3,queue_size=1)
    subIMU = rospy.Subscriber("hedge_imu_fusion", hedge_imu_fusion, imu_cb, queue_size=1)
    
    robot = HexapodC()

    while not rospy.is_shutdown():
        rospy.sleep(0.01)


