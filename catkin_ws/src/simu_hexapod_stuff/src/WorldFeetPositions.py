from numpy import sqrt, pi,arctan2,sin,cos
import rospy
from hexapodC import HexapodC
from sensor_msgs.msg import NavSatFix
from wgs2ned import WGS2NED
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point
from my_message.msg import LegPath, WorldFeetPlace

XPath = [[None for i in range(2)] for i in range(6)]
YPath = [[None for i in range(2)] for i in range(6)]
ZPath = [[None for i in range(2)] for i in range(6)]
StepS = 0; TurnPath = 0
def path_cb(msg):
    global StepS, TurnPath
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

    StepS = round(sqrt((abs(XPath[0][1]-XPath[0][0])/1000)**2 + (abs(YPath[0][1]-YPath[0][0])/1000)**2),3)
    
    yaw_rad = TurnPath * pi/180.0
    for i in range(6):
        existingAngle = arctan2(YPath[i][1],XPath[i][1])

        radius = sqrt(XPath[i][1]**2+YPath[i][1]**2)

        demandYaw = existingAngle + yaw_rad

        XPath[i][1] = radius*cos(demandYaw)
        YPath[i][1] = radius*sin(demandYaw)


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


if __name__ == '__main__':

    rospy.init_node("WorldFootPos")

    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)

    sub_path = rospy.Subscriber('/simple_hexapod/Legs_paths', LegPath, path_cb)
    subGPS = rospy.Subscriber("/simple_hexapod/fix", NavSatFix, nav_cb, queue_size=1)
    pubFeetPlace = rospy.Publisher("WorldFeetPlace", WorldFeetPlace, queue_size=1)

    FeetPlace = WorldFeetPlace()
    FeetPlace.XPlace = [None for i in range(6)]
    FeetPlace.YPlace = [None for i in range(6)]

    rospy.wait_for_message('/simple_hexapod/Legs_paths',LegPath)

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    BCP_x = 0.45
    BCP_y = 0.0
    cur_BCP_x = BCP_x + 0.1
    num = 2

    for i in range(1,3):

        # spawn_model_client(model_name='BCP'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')
        
        for j in [0,2,4]:
            FP_x = XPath[j][1]/1000 + BCP_x
            FP_y = YPath[j][1]/1000 + BCP_y
            # spawn_model_client(model_name='F'+str(j+1)+'P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(i),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
            FeetPlace.XPlace[j] = FP_x
            FeetPlace.YPlace[j] = FP_y

        BCP_x = BCP_x + 0.1

        # spawn_model_client(model_name='BC1P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')

        for j in [1,3,5]:
            FP_x = XPath[j][1]/1000 + BCP_x
            FP_y = YPath[j][1]/1000 + BCP_y
            # spawn_model_client(model_name='F'+str(j+1)+'P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(i),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
            FeetPlace.XPlace[j] = FP_x
            FeetPlace.YPlace[j] = FP_y

        BCP_x = BCP_x + 0.1

        pubFeetPlace.publish(FeetPlace) #publish world positions


    while not rospy.is_shutdown():
        print(BCP_x,cur_BCP_x,n)
        if abs(cur_BCP_x - n)/cur_BCP_x <= 0.01:

            cur_BCP_x = cur_BCP_x + 0.2
            num = num + 1

            for i in [0,2,4]:
                pass
                # delete_model_client(model_name='F'+str(i+1)+'P'+str(num-2))
                # delete_model_client(model_name='F'+str(i+2)+'P'+str(num-2))

            # spawn_model_client(model_name='BCP'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')
        
            for j in [0,2,4]:
                FP_x = XPath[j][1]/1000 + BCP_x
                FP_y = YPath[j][1]/1000 + BCP_y
                # spawn_model_client(model_name='F'+str(j+1)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(num),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
                FeetPlace.XPlace[j] = FP_x
                FeetPlace.YPlace[j] = FP_y

            BCP_x = BCP_x + 0.1

            # spawn_model_client(model_name='BC1P'+str(i),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='/BCP'+str(i),initial_pose=Pose(position=Point(BCP_x,BCP_y,0)),reference_frame='world')

            for j in [1,3,5]:
                FP_x = XPath[j][1]/1000 + BCP_x
                FP_y = YPath[j][1]/1000 + BCP_y
                # spawn_model_client(model_name='F'+str(j+1)+'P'+str(num),model_xml=open('/home/devlon/.gazebo/models/washer/model.sdf', 'r').read(),robot_namespace='F'+str(j)+'P'+str(num),initial_pose=Pose(position=Point(FP_x,FP_y,0)),reference_frame='world')
                FeetPlace.XPlace[j] = FP_x
                FeetPlace.YPlace[j] = FP_y

            BCP_x = BCP_x + 0.1

            pubFeetPlace.publish(FeetPlace) #publish world positions

        


            


    