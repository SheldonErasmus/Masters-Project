#!/usr/bin/env python3

import rospy
from math import atan2, pi, copysign
from marvelmind_nav.msg import hedge_pos_ang, hedge_imu_raw
from std_msgs.msg import Float32

n_prev = 0.0
e_prev = 0.0
d_prev = 0.0

n_cur=0; e_cur=0; d_cur=0; Yaw_meas_pair=0; Flag=0; counter = 98

def hedge_pos_ang_callback(msg):
    global n_prev, e_prev, d_prev, n_cur, e_cur, d_cur, Yaw_meas_pair, Flag, counter

    counter = counter + 1
    if counter == 100:
        counter = 0
        e_prev = e_cur
        n_prev = n_cur

    e_cur = msg.x_m*1000
    n_cur = msg.y_m*1000
    d_cur = -msg.z_m*1000

    if msg.angle>180:
        Yaw_meas_pair = msg.angle-360
    else:
        Yaw_meas_pair = msg.angle


    Flag = msg.flags
    #print(n_cur, e_cur, d_cur)

Cor_yaw_cur = 0
def imu_cb(msg):
    global Est_yaw_next,Cor_yaw_cur

    if msg.gyro_z>-80 and msg.gyro_z<0:
        Gyro_z = 0
    else:
        Gyro_z = msg.gyro_z*0.0175

    mc1 = 0.5
    mc2 = 0.07
    Ts = 1/100

    Est_yaw_cur = Est_yaw_next

    Cor_yaw_cur = Est_yaw_cur

    n_difference = n_cur-n_prev
    e_difference = e_cur-e_prev
    if abs(n_difference) > 200 or abs(e_difference) > 200: 
        Yaw_meas = atan2(n_difference,e_difference)*180/pi
        dif1 = Yaw_meas - Est_yaw_cur
        if abs(dif1)>180:
            dif1 = dif1 - copysign(360,dif1)
        Cor_yaw_cur = Cor_yaw_cur + mc1*(dif1)
    if 1:
        dif2 = Yaw_meas_pair - Est_yaw_cur
        if abs(dif2)>180:
            dif2 = dif2 - copysign(360,dif2)
        Cor_yaw_cur = Cor_yaw_cur + mc2*(dif2)
    
    if abs(Cor_yaw_cur)>180:
        Cor_yaw_cur = Cor_yaw_cur - copysign(360,Cor_yaw_cur)

    Est_yaw_next = Cor_yaw_cur + Ts*Gyro_z
    pubyaw.publish(data=Cor_yaw_cur)
    print(Cor_yaw_cur,Est_yaw_next,Ts*Gyro_z)

if __name__ == '__main__':
    rospy.init_node("YawPub")
    
    pubyaw = rospy.Publisher('/simple_hexapod/calculated_yaw', Float32 ,queue_size=1)
    subIndoorGPS = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
    rospy.wait_for_message("hedge_pos_ang", hedge_pos_ang)
    Est_yaw_next = Yaw_meas_pair
    subIMU = rospy.Subscriber("hedge_imu_raw", hedge_imu_raw, imu_cb, queue_size=1)

    while not rospy.is_shutdown():
        pass
