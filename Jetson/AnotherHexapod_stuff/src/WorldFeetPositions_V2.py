#!/usr/bin/env python3

from numpy import sqrt, pi,arctan2,sin,cos
import rospy
from my_message.msg import LegPath, WorldFeetPlace
from std_msgs.msg import Float32
from marvelmind_nav.msg import hedge_pos_ang

XPath = [[None for i in range(3)] for i in range(6)]
YPath = [[None for i in range(3)] for i in range(6)]
ZPath = [[None for i in range(3)] for i in range(6)]
TurnPath = [None for i in range(3)]
StepS_x = 0; StepS_y = 0
FeetChangedFlag = 0
def path_cb(msg):
    global StepS_x, StepS_y, FeetChangedFlag
    FeetChangedFlag = 1
    for col in [0,3,6]:
        XPath[0][col//3]=msg.PathL0x[col]
        XPath[1][col//3]=msg.PathL1x[col]
        XPath[2][col//3]=msg.PathL2x[col]
        XPath[3][col//3]=msg.PathL3x[col]
        XPath[4][col//3]=msg.PathL4x[col]
        XPath[5][col//3]=msg.PathL5x[col]

        YPath[0][col//3]=msg.PathL0y[col]
        YPath[1][col//3]=msg.PathL1y[col]
        YPath[2][col//3]=msg.PathL2y[col]
        YPath[3][col//3]=msg.PathL3y[col]
        YPath[4][col//3]=msg.PathL4y[col]
        YPath[5][col//3]=msg.PathL5y[col]

        ZPath[0][col//3]=msg.PathL0z[col]
        ZPath[1][col//3]=msg.PathL1z[col]
        ZPath[2][col//3]=msg.PathL2z[col]
        ZPath[3][col//3]=msg.PathL3z[col]
        ZPath[4][col//3]=msg.PathL4z[col]
        ZPath[5][col//3]=msg.PathL5z[col]

        TurnPath[col//3] = msg.PathAng[col]

    # StepS = round(sqrt((abs(XPath[0][1]-XPath[0][0])/1000)**2 + (abs(YPath[0][1]-YPath[0][0])/1000)**2),3)
        
    for j in range(3):
        for i in range(6):
            existingAngle = arctan2(YPath[i][j],XPath[i][j])

            radius = sqrt(XPath[i][j]**2+YPath[i][j]**2)

            if i == 1 or i == 3 or i == 5:
                yaw_rad = -TurnPath[0] * pi/180.0 if j == 2 else (-TurnPath[2] * pi/180.0 if j == 0 else -TurnPath[1] * pi/180.0)
            else:
                yaw_rad = TurnPath[j] * pi/180.0

            demandYaw = existingAngle + yaw_rad

            XPath[i][j] = radius*cos(demandYaw)
            YPath[i][j] = radius*sin(demandYaw)


n_cur = 0.0
e_cur = 0.0
d_cur = 0.0
def hedge_pos_ang_callback(msg):
    global n_cur, e_cur, d_cur
    e_cur = msg.x_m
    n_cur = msg.y_m
    d_cur = -msg.z_m

FeetOnFloorFlag = 1
feetOnFloor = [1,5]
def FeetOnFloor_cb(msg):
    global FeetOnFloorFlag,feetOnFloor
    FeetOnFloorFlag = 1
    if feetOnFloor[0] == 2: feetOnFloor = [1,5]
    elif feetOnFloor[0] == 1: feetOnFloor = [2,4]

yaw=0
def yaw_cb(msg):
    global yaw
    yaw = msg.data*pi/180

if __name__ == '__main__':

    rospy.init_node("WorldFootPos")

    sub_path = rospy.Subscriber('/simple_hexapod/Legs_paths', LegPath, path_cb)
    subGPS = rospy.Subscriber("hedge_pos_ang", hedge_pos_ang, hedge_pos_ang_callback, queue_size=1)
    subyaw = rospy.Subscriber("/simple_hexapod/calculated_yaw", Float32, yaw_cb, queue_size=1)
    FeetOnFloorFlag_sub = rospy.Subscriber("/FeetOnFloorFlag",Float32, FeetOnFloor_cb)
    pubFeetPlace = rospy.Publisher("WorldFeetPlace", WorldFeetPlace, queue_size=1)

    FeetPlace = WorldFeetPlace()
    FeetPlace.XPlace = [None for i in range(6)]
    FeetPlace.YPlace = [None for i in range(6)]

    rospy.wait_for_message('/simple_hexapod/Legs_paths',LegPath)

    num = 0
    for k in [1,3,5]:
        StartFlag = 1
        FeetPlace.XPlace[k]  = (XPath[k][1]/1000)*cos(yaw) - (YPath[k][1]/1000)*sin(yaw) + e_cur
        FeetPlace.YPlace[k]  = (XPath[k][1]/1000)*sin(yaw) + (YPath[k][1]/1000)*cos(yaw) + (n_cur)

    LoopRange = [1,3,5]

    while not rospy.is_shutdown():

        if FeetOnFloorFlag == 1:
            FeetOnFloorFlag = 0

            if StartFlag == 0:
                for k in LoopRange:
                    FeetPlace.XPlace[k]  = (XPath[k][2]/1000)*cos(yaw) - (YPath[k][2]/1000)*sin(yaw) + e_cur
                    FeetPlace.YPlace[k]  = (XPath[k][2]/1000)*sin(yaw) + (YPath[k][2]/1000)*cos(yaw) + (n_cur)
            StartFlag = 0

            phi_LB = -arctan2((XPath[feetOnFloor[1]][0]-XPath[feetOnFloor[0]][0]),(YPath[feetOnFloor[1]][0]-YPath[feetOnFloor[0]][0]))
            phi_LW = -arctan2((FeetPlace.XPlace[feetOnFloor[1]]-FeetPlace.XPlace[feetOnFloor[0]]),(FeetPlace.YPlace[feetOnFloor[1]]-FeetPlace.YPlace[feetOnFloor[0]]))

            if feetOnFloor[0] == 1: LoopRange = [0,2,4]
            elif feetOnFloor[0] == 2: LoopRange = [1,3,5]

            num = num +1

            for k in LoopRange:
                deltaX_B = XPath[k][2]-XPath[feetOnFloor[0]][0]
                deltaY_B = YPath[k][2]-YPath[feetOnFloor[0]][0]

                deltaX_L = cos(phi_LB)*deltaX_B + sin(phi_LB)*deltaY_B
                deltaY_L = -sin(phi_LB)*deltaX_B + cos(phi_LB)*deltaY_B

                deltaX_W = cos(phi_LW)*deltaX_L - sin(phi_LW)*deltaY_L
                deltaY_W = sin(phi_LW)*deltaX_L + cos(phi_LW)*deltaY_L

                FeetPlace.XPlace[k] = deltaX_W/1000 + FeetPlace.XPlace[feetOnFloor[0]]
                FeetPlace.YPlace[k] = deltaY_W/1000 + FeetPlace.YPlace[feetOnFloor[0]]

            pubFeetPlace.publish(FeetPlace) #publish world positions
                


        


            


    