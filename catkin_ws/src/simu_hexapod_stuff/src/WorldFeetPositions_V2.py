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

XPath = [[None for i in range(3)] for i in range(6)]
YPath = [[None for i in range(3)] for i in range(6)]
ZPath = [[None for i in range(3)] for i in range(6)]
StepS_x = 0; StepS_y = 0; TurnPath = 0
FeetChangedFlag = 0
def path_cb(msg):
    global StepS_x, StepS_y, TurnPath, FeetChangedFlag
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

        TurnPath = msg.PathAng[6]

    # StepS = round(sqrt((abs(XPath[0][1]-XPath[0][0])/1000)**2 + (abs(YPath[0][1]-YPath[0][0])/1000)**2),3)
        
    yaw_rad = TurnPath * pi/180.0
    for i in range(6):
        existingAngle = arctan2(YPath[i][2],XPath[i][2])

        radius = sqrt(XPath[i][2]**2+YPath[i][2]**2)

        demandYaw = existingAngle + yaw_rad

        XPath[i][2] = radius*cos(demandYaw)
        YPath[i][2] = radius*sin(demandYaw)

    StepS_x = round(abs(XPath[0][2]-XPath[0][0])/1000,3)
    StepS_y = round((YPath[0][2]-YPath[0][0])/1000,3)

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
    if feetOnFloor[0] == 2: feetOnFloor = [1,5]
    elif feetOnFloor[0] == 1: feetOnFloor = [2,4]

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

    num = -1
    for k in range(6):
        #num = num +1
        FeetPlace.XPlace[k]  = (XPath[k][1]/1000)*cos(yaw) - (YPath[k][1]/1000)*sin(yaw) + n
        FeetPlace.YPlace[k]  = (XPath[k][1]/1000)*sin(yaw) + (YPath[k][1]/1000)*cos(yaw) + (-e)
        #spawn_model_client(model_name='F'+str(k)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(k)+'P'+str(num),initial_pose=Pose(position=Point(FeetPlace.XPlace[k],FeetPlace.YPlace[k],0)),reference_frame='world')

    
    while not rospy.is_shutdown():

        if FeetOnFloorFlag == 1:
            FeetOnFloorFlag = 0

            phi_LB = arctan2((XPath[feetOnFloor[1]][0]-XPath[feetOnFloor[0]][0]),(YPath[feetOnFloor[1]][0]-YPath[feetOnFloor[0]][0]))
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

                spawn_model_client(model_name='F'+str(k)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(k)+'P'+str(num),initial_pose=Pose(position=Point(FeetPlace.XPlace[k],FeetPlace.YPlace[k],0)),reference_frame='world')
                


        


            


    