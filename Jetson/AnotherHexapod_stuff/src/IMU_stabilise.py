#!/usr/bin/env python3

from math import pi
import rospy
from hexapodC import HexapodC
from ToEulerAngles import ToEulerAng
import time
from marvelmind_nav.msg import hedge_imu_fusion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

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

        # if i0 > 10:
        #     i0 = 10
        # elif i0 < -10:
        #     i0 = -10

        self.u = p0 + i0 + d0

        self.i1 = i0
        self.d1 = d0
        self.e1 = e0

        return self.u
        

measure_roll = 0.0; measure_pitch = 0.0
ref_roll = 0.0; ref_pitch = 0.0
roll_input = 0.0; pitch_input = 0.0
timestamp = 0.0

def imu_cb(msg):
    global measure_roll,measure_pitch,roll_input,pitch_input,IMU_toggle, timestamp
    (measure_roll,measure_pitch,yaw) = ToEulerAng(msg.qx,msg.qy,msg.qz,msg.qw)
    timestamp = msg.timestamp_ms

    if mode == 2 and IMU_toggle == 1: 
        roll_input = rollController.PIDUpdate(ref_roll,measure_roll*180/pi)
        pitch_input = pitchController.PIDUpdate(ref_pitch,measure_pitch*180/pi)
        pubRollPitch.publish(x = pitch_input,y=-roll_input)
    elif IMU_toggle == 0:
        roll_input0 = rollController.PIDUpdate(-roll_input,measure_roll*180/pi)
        pitch_input0 = pitchController.PIDUpdate(-pitch_input,measure_pitch*180/pi)
        pubRollPitch.publish(x = pitch_input0,y=-roll_input0)
        if (abs(roll_input0) < 1.2) and (abs(pitch_input0) < 1.2 ):
            IMU_toggle = -1
    elif IMU_toggle == -1:
        pubRollPitch.publish(x = 0.0,y=-0.0)

    print(measure_roll*180/pi,measure_pitch*180/pi,roll_input,pitch_input)
    #print(roll_input)

mode = -1
def modeSelect_cb(msg):
    global mode
    mode = msg.data
rosSubModeSelect = rospy.Subscriber("/mode_selected", Float32,modeSelect_cb)

IMU_toggle = -1
def IMU_toggle_cb(msg):
    global IMU_toggle
    IMU_toggle = msg.data
rosSubModeSelect = rospy.Subscriber("/IMU_toggle", Float32,IMU_toggle_cb)

if __name__ == '__main__':

    rollController = PIDcontroller(0.0, 1/1, 0.0, 0.0, 1/100)
    pitchController = PIDcontroller(0.0, 1/1, 0.0, 0.0, 1/100)

    rospy.init_node("IMU_stabilise")
    pubRollPitch = rospy.Publisher("RollPitch_input",Vector3,queue_size=1)
    subIMU = rospy.Subscriber("hedge_imu_fusion", hedge_imu_fusion, imu_cb, queue_size=1)
    
    robot = HexapodC()

    fileName = 'IMUdataRec2.csv'
    try:
        with open(fileName,"w") as file:
            file.write("timestamp,roll_input,pitch_input,measure_roll,measure_pitch\n")

            while not rospy.is_shutdown():
                file.write(str(timestamp) + "," + str(roll_input) + "," + str(pitch_input) + "," + str(measure_roll*180/pi) + "," + str(measure_pitch*180/pi) + "\n")
                rospy.sleep(0.01)
    except KeyboardInterrupt:
        pass


