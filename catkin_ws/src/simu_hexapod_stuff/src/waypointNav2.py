#!/usr/bin/env python3

import rospy
from hexapodC import HexapodC
from sensor_msgs.msg import NavSatFix, Imu
from wgs2ned import WGS2NED
from math import atan2, sqrt, cos, sin, pi, copysign, isclose
from numpy import array
from ToEulerAngles import ToEulerAng
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point
import time
from GenerateMotionProfile import TrapezoidalProfile, StepProfile
from my_message.msg import PathVar_n_cmdVel
from std_msgs.msg import Float32

#Global variables
startupFlag = 1
refLat = 0.0
refLon = 0.0
refAlt = 0.0
n = 0.0
e = 0.0
d = 0.0
curr_Head_hex = 0.0; hex_roll = 0.0; hex_pitch = 0.0 #Added 26Jan


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

def imu_cb(msg):
    global curr_Head_hex,hex_roll,hex_pitch
    (hex_roll,hex_pitch,curr_Head_hex) = ToEulerAng(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w) #Added 26Jan
    curr_Head_hex = -curr_Head_hex

NavMode = 0
once = 1
def NavMode_cb(msg):
    global NavMode, once
    NavMode = msg.data
    once = 1

def integrator(u,timestep):
    global y
    y = y + timestep*u
    return y

if __name__ == '__main__':
    rospy.init_node("WayP_Nav")

    robot = HexapodC()
    #trapez = TrapezoidalProfile() #11Nov
    stepP = StepProfile(0.1,150) #Added 26Jan
    
    model_coordinates = rospy.ServiceProxy( '/gazebo/get_model_state', GetModelState)
    subGPS = rospy.Subscriber(robot.ns + "fix", NavSatFix, nav_cb, queue_size=1)
    subIMU = rospy.Subscriber(robot.ns + "imu", Imu, imu_cb, queue_size=1)
    pub = rospy.Publisher('/simple_hexapod/changed_vel_path_var', PathVar_n_cmdVel, queue_size=1)
    Nav_sub = rospy.Subscriber('/Nav_mode',Float32,NavMode_cb,queue_size=1)

    PathVelmsg = PathVar_n_cmdVel()
    PathVelmsg.Name = "Waypoint"
    
    Esrc = float(input("Enter Esrc: "))
    Nsrc = float(input("Enter Nsrc: ") )
    Edest = float(input("Enter Edest: "))
    Ndest = float(input("Enter Ndest: ")) 
    DoWaypointFlag = 1
    StartOfWaypoint = 1
    index = 0
    total_items = 2
  
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_model_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    spawn_model_client(model_name='can1',model_xml=open('/home/devlon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can1',initial_pose=Pose(position=Point(Nsrc,-Esrc,0)),reference_frame='world')
    spawn_model_client(model_name='can2',model_xml=open('/home/devlon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can2',initial_pose=Pose(position=Point(Ndest,-Edest,0)),reference_frame='world')

    cannum = 2
    # flag = 1
    # prevTime = time.time()

    # Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))
    # L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)

    # ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
    # y = ct[1]
    # startPos = ct[0] #11Nov

    #trapez.GenerateProfile(startPos,L_t,150) #11Nov
    #stepP.GenerateProfile(startPos,L_t,0) #Added 26Jan
    # TrapezstartTime = time.time() #11Nov

    while not rospy.is_shutdown():

        if DoWaypointFlag == 1 and NavMode == 2:

            # currTime = time.time()
            # stepTime = currTime - prevTime
            # prevTime = currTime

            if StartOfWaypoint == 1:
                StartOfWaypoint = 0
                flag = 1

                Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))
                L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)

                TrapezstartTime = time.time()

            if flag == 1:
                ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
                ct_dist = ct[0]
                ct_err = ct[1]

            if ct_dist < L_t:
                        
                ct_err_ref = 0.0
                err_ref = ct_err_ref - ct_err #ct_err Nie integrasie gebruik nie
                Ky = 0.7 #0.3 
                Head_ref = Head_t + Ky*err_ref
                Head_command = -(Head_ref - curr_Head_hex)

                #xdot = trapez.F(time.time()-TrapezstartTime) #11Nov
                stepP.GenerateProfile(ct_dist,L_t,(time.time()-TrapezstartTime)) #Added 26Jan
                xdot = stepP.F(time.time()-TrapezstartTime) #Added 26Jan
                V_hexTot = xdot #11Nov

                if abs(Head_command*180/pi) >= 15:
                    PathVelmsg.linear.x=V_hexTot; PathVelmsg.angular.z=copysign(15,Head_command)
                    pub.publish(PathVelmsg) #robot.set_walk_velocity(V_hexTot,0,copysign(15,Head_command))
                else:
                    PathVelmsg.linear.x=V_hexTot; PathVelmsg.angular.z=Head_command*180/pi
                    pub.publish(PathVelmsg) #robot.set_walk_velocity(V_hexTot,0,Head_command*180/pi)

                # ydot = V_hexTot*(curr_Head_hex-Head_t)
                # y = integrator(ydot,stepTime)

                #rospy.loginfo([ct_err, y, Head_ref, curr_Head_hex, Head_command])
                rospy.loginfo([V_hexTot,-TrapezstartTime+time.time()])
                
            else:
                if flag == 1:
                    PathVelmsg.linear.x=0; PathVelmsg.angular.z=0
                    pub.publish(PathVelmsg) #robot.set_walk_velocity(0.0,0,0)
                    Esrc = Edest
                    Nsrc = Ndest
                    Edest = float(input("Enter new Edest: "))
                    Ndest = float(input("Enter new Ndest: ") )
                    Head_t = atan2((Edest-Esrc),(Ndest-Nsrc))

                    cannum = cannum +1
                    spawn_model_client(model_name='can'+str(cannum) ,model_xml=open('/home/devlon/.gazebo/models/cricket_ball/model.sdf', 'r').read(),robot_namespace='/can'+str(cannum),initial_pose=Pose(position=Point(Ndest,-Edest,0)),reference_frame='world')
                    flag = 0

                if abs(Head_t - curr_Head_hex) > 0.01:
                    if abs((Head_t - curr_Head_hex)*180/pi) >= 15:
                        PathVelmsg.linear.x=0; PathVelmsg.angular.z=copysign(15,-(Head_t - curr_Head_hex))
                        pub.publish(PathVelmsg) #robot.set_walk_velocity(0.0,0,copysign(15,-(Head_t - curr_Head_hex)))
                    else:
                        PathVelmsg.linear.x=0; PathVelmsg.angular.z=-(Head_t - curr_Head_hex)*180/pi
                        pub.publish(PathVelmsg) #robot.set_walk_velocity(0.0,0,-(Head_t - curr_Head_hex)*180/pi)
                else:
                    PathVelmsg.linear.x=0; PathVelmsg.angular.z=0
                    pub.publish(PathVelmsg) #robot.set_walk_velocity(0.0,0,0)
                    L_t = sqrt((Ndest-Nsrc)**2+(Edest-Esrc)**2)
                    flag = 1

                    # ct = array([[cos(Head_t), sin(Head_t)],[-sin(Head_t), cos(Head_t)]]) @ array([[n-Nsrc],[e-Esrc]])
                    # startPos = ct[0] #11Nov

                    # trapez.GenerateProfile(startPos,L_t,150) #11Nov
                    # stepP.GenerateProfile(startPos,L_t,150) #Added 26Jan
                    TrapezstartTime = time.time() #11Nov

        elif NavMode == 1:
            if once == 1:
                once = 0
                onceStop = 1
                headControlON = 1
                Desired_Head = -float(input("Enter desired heading: "))*pi/180
            Gain = 1
            ref_head = Desired_Head
            Head_command = -Gain*(ref_head - curr_Head_hex)

            if abs(Head_command) > pi:
                Head_command = Head_command - copysign(2*pi,Head_command)

            if abs(Head_command*180/pi) < 0.5:
                headControlON = 0
            if abs(Head_command*180/pi) > 2:
                headControlON = 1
            
            if headControlON == 1: 
                onceStop = 1
                if abs(Head_command*180/pi) >= 15:
                    PathVelmsg.linear.x=float('nan'); PathVelmsg.angular.z=copysign(15,Head_command)
                    pub.publish(PathVelmsg) #robot.set_walk_velocity(V_hexTot,0,copysign(15,Head_command))
                else:
                    PathVelmsg.linear.x=float('nan'); PathVelmsg.angular.z=Head_command*180/pi
                    pub.publish(PathVelmsg)
            elif onceStop == 1:
                onceStop = 0
                PathVelmsg.linear.x=float('nan'); PathVelmsg.angular.z=0.0*180/pi
                pub.publish(PathVelmsg)

        rospy.sleep(0.1) 