from numpy import sqrt, pi,arctan2,sin,cos
import rospy
from hexapodC import HexapodC
from sensor_msgs.msg import NavSatFix, Imu
from ToEulerAngles import ToEulerAng
from wgs2ned import WGS2NED
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point
from my_message.msg import LegPath, WorldFeetPlace
from std_msgs.msg import Float32

XPath = [[None for i in range(2)] for i in range(6)]
YPath = [[None for i in range(2)] for i in range(6)]
ZPath = [[None for i in range(2)] for i in range(6)]
StepS_x = 0; StepS_y = 0; TurnPath = 0
FeetChangedFlag = 0
def path_cb(msg):
    global StepS_x, StepS_y, TurnPath, FeetChangedFlag
    FeetChangedFlag = 1
    for col in [0,6]:
        XPath[0][col%5]=msg.PathL0x[col]
        XPath[1][col%5]=msg.PathL1x[col]
        XPath[2][col%5]=msg.PathL2x[col]
        XPath[3][col%5]=msg.PathL3x[col]
        XPath[4][col%5]=msg.PathL4x[col]
        XPath[5][col%5]=msg.PathL5x[col]

        YPath[0][col%5]=msg.PathL0y[col]
        YPath[1][col%5]=msg.PathL1y[col]
        YPath[2][col%5]=msg.PathL2y[col]
        YPath[3][col%5]=msg.PathL3y[col]
        YPath[4][col%5]=msg.PathL4y[col]
        YPath[5][col%5]=msg.PathL5y[col]

        ZPath[0][col%5]=msg.PathL0z[col]
        ZPath[1][col%5]=msg.PathL1z[col]
        ZPath[2][col%5]=msg.PathL2z[col]
        ZPath[3][col%5]=msg.PathL3z[col]
        ZPath[4][col%5]=msg.PathL4z[col]
        ZPath[5][col%5]=msg.PathL5z[col]

        TurnPath = msg.PathAng[6]

    # StepS = round(sqrt((abs(XPath[0][1]-XPath[0][0])/1000)**2 + (abs(YPath[0][1]-YPath[0][0])/1000)**2),3)
        
    yaw_rad = TurnPath * pi/180.0
    for i in range(6):
        existingAngle = arctan2(YPath[i][1],XPath[i][1])

        radius = sqrt(XPath[i][1]**2+YPath[i][1]**2)

        demandYaw = existingAngle + yaw_rad

        XPath[i][1] = radius*cos(demandYaw)
        YPath[i][1] = radius*sin(demandYaw)

    StepS_x = round(abs(XPath[0][1]-XPath[0][0])/1000,3)
    StepS_y = round((YPath[0][1]-YPath[0][0])/1000,3)

n = 0.0
e = 0.0
d = 0.0
def nav_cb(msg):
    global n, e, d
    object_coordinates = model_coordinates("simple_hexapod","")
    n = object_coordinates.pose.position.x
    e = -object_coordinates.pose.position.y
    d = -object_coordinates.pose.position.z

FeetOnFloorFlag = 1
feetOnFloor = [1,5]
def FeetOnFloor_cb(msg):
    global FeetOnFloorFlag,feetOnFloor
    FeetOnFloorFlag = 1
    if feetOnFloor[0] == 4: feetOnFloor = [1,5]
    elif feetOnFloor[0] == 1: feetOnFloor = [4,2]

roll=0;pitch=0;yaw=0
def imu_cb(msg):
    global roll,pitch,yaw
    (roll,pitch,yaw) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)

if __name__ == '__main__':

    rospy.init_node("WorldFootPos")

    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)

    sub_path = rospy.Subscriber('/simple_hexapod/Legs_paths', LegPath, path_cb)
    subGPS = rospy.Subscriber("/simple_hexapod/fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber("/simple_hexapod/imu", Imu, imu_cb, queue_size=1)
    FeetOnFloorFlag_sub = rospy.Subscriber("/FeetOnFloorFlag",Float32, FeetOnFloor_cb)
    pubFeetPlace = rospy.Publisher("WorldFeetPlace", WorldFeetPlace, queue_size=1)

    FeetPlace = WorldFeetPlace()
    FeetPlace.XPlace = [None for i in range(6)]
    FeetPlace.YPlace = [None for i in range(6)]

    rospy.wait_for_message('/simple_hexapod/Legs_paths',LegPath)

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    for k in range(6):
        FeetPlace.XPlace[k]  = (XPath[k][1]/1000)*cos(yaw) - (YPath[k][1]/1000)*sin(yaw) + n
        FeetPlace.YPlace[k]  = (XPath[k][1]/1000)*sin(yaw) + (YPath[k][1]/1000)*cos(yaw) + (-e)

    while not rospy.is_shutdown():

        if FeetOnFloorFlag == 1:
            FeetOnFloorFlag = 0

            phi_L = arctan2((XPath[feetOnFloor[1]][1]-XPath[feetOnFloor[0]][1]),(YPath[feetOnFloor[1]][1]-YPath[feetOnFloor[0]][1]))

            if feetOnFloor[0] == 1: LoopRange = [0,2,4]
            elif feetOnFloor[0] == 4: LoopRange = [1,3,5]

            for k in LoopRange:
                deltaX_B = XPath[k][1]-XPath[feetOnFloor[0]][1]
                deltaY_B = YPath[k][1]-YPath[feetOnFloor[0]][1]

                deltaX_L = cos(phi_L)*deltaX_B + sin(phi_L)*deltaY_B
                deltaY_L = -sin(phi_L)*deltaX_B + cos(phi_L)*deltaY_B

                #FootX_L = 


        


            


    