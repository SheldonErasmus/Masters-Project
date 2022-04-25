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


startupFlag = 1
refLat = 0.0
refLon = 0.0
refAlt = 0.0
n = 0.0
e = 0.0
d = 0.0
def nav_cb(msg):
    global startupFlag, refLat, refLon, refAlt, n, e, d
    # if startupFlag:
    #     startupFlag = 0
    #     refLat = msg.latitude
    #     refLon = msg.longitude
    #     refAlt = msg.altitude
    # Lat = msg.latitude
    # Lon = msg.longitude
    # Alt = msg.altitude
    # (n,e,d) = WGS2NED(refLat,refLon,refAlt,Lat,Lon,Alt)
    object_coordinates = model_coordinates("simple_hexapod","")
    n = object_coordinates.pose.position.x
    e = -object_coordinates.pose.position.y
    d = -object_coordinates.pose.position.z

FeetOnFloorFlag = 0
def FeetOnFloor_cb(msg):
    global FeetOnFloorFlag
    FeetOnFloorFlag = FeetOnFloorFlag + msg.data

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

    BCP_x = 0.0
    BCP_y = 0.0
    cur_BCP_x = BCP_x + 0.1
    num = 2
    FeetChangedFlag = 0
    nprev = n
    eprev = e

    for i in range(1,3):
        l = n if i == 1 else BCP_x
        k = StepS_x/2 if i == 1 else StepS_x
        BCP_x = l + k

        l = -e if i == 1 else BCP_y
        k = StepS_y/2 if i == 1 else StepS_y
        BCP_y = l + k

        # spawn_model_client(model_name='BCP'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')
        
        for j in [0,2,4]:
            FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
            FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y
            spawn_model_client(model_name='F'+str(j+1)+'P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(i),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
            FeetPlace.XPlace[j] = FP_x
            FeetPlace.YPlace[j] = FP_y

        BCP_x = BCP_x + StepS_x
        BCP_y = BCP_y + StepS_y

        # spawn_model_client(model_name='BC1P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')

        for j in [1,3,5]:
            FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
            FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y
            spawn_model_client(model_name='F'+str(j+1)+'P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(i),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
            FeetPlace.XPlace[j] = FP_x
            FeetPlace.YPlace[j] = FP_y

        pubFeetPlace.publish(FeetPlace) #publish world positions


    while not rospy.is_shutdown():
        print(BCP_x,cur_BCP_x,n,FeetOnFloorFlag)
        if FeetOnFloorFlag >= 2: #abs(cur_BCP_x - n)/cur_BCP_x <= 0.01

            FeetOnFloorFlag = 0
            cur_BCP_x = cur_BCP_x + 0.2
            num = num + 1
            nprev = n
            eprev = e

            for i in [0,2,4]:
                pass
                delete_model_client(model_name='F'+str(i+1)+'P'+str(num-2))
                delete_model_client(model_name='F'+str(i+2)+'P'+str(num-2))

            BCP_x = n + 3*StepS_x
            BCP_y = e + 3*StepS_y

            # spawn_model_client(model_name='BCP'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')
        
            for j in [0,2,4]:
                FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
                FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y    
                spawn_model_client(model_name='F'+str(j+1)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(num),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
                FeetPlace.XPlace[j] = FP_x
                FeetPlace.YPlace[j] = FP_y

            BCP_x = BCP_x + StepS_x
            BCP_y = BCP_y + StepS_y

            # spawn_model_client(model_name='BC1P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')

            for j in [1,3,5]:
                FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
                FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y 
                spawn_model_client(model_name='F'+str(j+1)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(num),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
                FeetPlace.XPlace[j] = FP_x
                FeetPlace.YPlace[j] = FP_y

            pubFeetPlace.publish(FeetPlace) #publish world positions


        # if FeetChangedFlag == 1:
        #     FeetChangedFlag = 0
        #     for i in [0,2,4]:
        #         pass
        #         delete_model_client(model_name='F'+str(i+1)+'P'+str(num))
        #         delete_model_client(model_name='F'+str(i+2)+'P'+str(num))
        #         delete_model_client(model_name='F'+str(i+1)+'P'+str(num-1))
        #         delete_model_client(model_name='F'+str(i+2)+'P'+str(num-1))

        #     for i,m in zip(range(1,3),[num-1,num]):
        #         if round(nprev,2) == 0:
        #             l = nprev if i == 1 else BCP_x
        #             k = StepS_x/2 if i == 1 else StepS_x
        #         else:
        #             l = nprev
        #             k = 1*StepS_x if i == 1 else 3*StepS_x
        #         BCP_x = l + k

        #         if round(eprev,2) == 0:
        #             l = eprev if i == 1 else BCP_y
        #             k = StepS_y/2 if i == 1 else StepS_y
        #         else:
        #             l = eprev
        #             k = 1*StepS_y if i == 1 else 3*StepS_y
        #         BCP_y = l + k

        #         # spawn_model_client(model_name='BCP'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')
            
        #         for j in [0,2,4]:
        #             FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
        #             FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y 
        #             spawn_model_client(model_name='F'+str(j+1)+'P'+str(m),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(m),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
        #             FeetPlace.XPlace[j] = FP_x
        #             FeetPlace.YPlace[j] = FP_y

        #         BCP_x = BCP_x + StepS_x
        #         BCP_y = BCP_y + StepS_y

        #         # spawn_model_client(model_name='BC1P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')

        #         for j in [1,3,5]:
        #             FP_x = (XPath[j][1]/1000)*cos(yaw) - (YPath[j][1]/1000)*sin(yaw) + BCP_x
        #             FP_y = (XPath[j][1]/1000)*sin(yaw) + (YPath[j][1]/1000)*cos(yaw) + BCP_y 
        #             spawn_model_client(model_name='F'+str(j+1)+'P'+str(m),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(m),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
        #             FeetPlace.XPlace[j] = FP_x
        #             FeetPlace.YPlace[j] = FP_y

        #         pubFeetPlace.publish(FeetPlace) #publish world positions
        


            


    