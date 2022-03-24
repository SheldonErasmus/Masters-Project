from interpo import *
from IK import IK
from FK import FK03_inbody
import rospy
from std_msgs.msg import Float32, Float64, Float64MultiArray
from my_message.msg import LegPath, thetaMessage
from geometry_msgs.msg import Twist, Vector3
import time

def millis():
    return time.time()*1000

#Joint Angle Variables for SetNextpathPoint Mode
theta1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
theta2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
theta3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#Joint Angle Variables for SetAngle Mode
A_theta1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
A_theta2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
A_theta3 = [-1.5, -1.5, -1.5, -1.5, -1.5, -1.5]
#Joint Angle Variables for Teleop demo Mode
T_theta1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
T_theta2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
T_theta3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

rospy.init_node('TeensyEquivalent')

def angle_command_cb(msg):
    A_theta1[0] = msg.th1_1
    A_theta1[1] = msg.th1_2
    A_theta1[2] = msg.th1_3
    A_theta1[3] = msg.th1_4
    A_theta1[4] = msg.th1_5
    A_theta1[5] = msg.th1_6

    A_theta2[0] = msg.th2_1
    A_theta2[1] = msg.th2_2
    A_theta2[2] = msg.th2_3
    A_theta2[3] = msg.th2_4
    A_theta2[4] = msg.th2_5
    A_theta2[5] = msg.th2_6

    A_theta3[0] = msg.th3_1
    A_theta3[1] = msg.th3_2
    A_theta3[2] = msg.th3_3
    A_theta3[3] = msg.th3_4
    A_theta3[4] = msg.th3_5
    A_theta3[5] = msg.th3_6
rosSub = rospy.Subscriber('/simple_hexapod/Th_position_controller/command',thetaMessage,angle_command_cb)

tx=0.0; ty=0.0; tz=0.0
troll=0.0; tpitch=0.0; tyaw=0.0
def teleop_cb(msg):
    global tx, ty, tz, troll, tpitch, tyaw
    #lean x direction
    if (msg.linear.x > 0 and msg.linear.x <= 0.5):
        tx = tx + 1
    elif (msg.linear.x < 0 and msg.linear.x >= -0.5):
        tx = tx - 1
    #lean y direction
    if (msg.linear.y > 0 and msg.linear.y <= 0.5):
        ty = ty + 1
    elif (msg.linear.y < 0 and msg.linear.y >= -0.5):
        ty = ty - 1
    #Change height
    if (msg.linear.z > 0):
        tz = tz + 1
    elif (msg.linear.z < 0):
        tz = tz - 1
    #Yaw movement
    if (msg.angular.z > 0):
        tyaw = tyaw + 0.25
    elif (msg.angular.z < 0):
        tyaw = tyaw - 0.25
    #Roll movement
    if (msg.linear.x > 0.5):
        troll = troll + 0.25
    elif (msg.linear.x < -0.5):
        troll = troll - 0.25
    #Pitch movement
    if (msg.linear.y > 0.5):
        tpitch = tpitch + 0.25
    elif (msg.linear.y < -0.5):
        tpitch = tpitch - 0.25
rosSubTeleop = rospy.Subscriber("/cmd_vel",Twist,teleop_cb)

Pathsize = 7
XPath = [[None for i in range(Pathsize*2-2)] for i in range(6)]
YPath = [[None for i in range(Pathsize*2-2)] for i in range(6)]
ZPath = [[None for i in range(Pathsize*2-2)] for i in range(6)]
TurnPath = [None for i in range(Pathsize*2-2)]
dt=0
startSetPath = 0
def legpath_cb(msg):
    global dt, startSetPath

    for col in range(Pathsize*2-2):
        XPath[0][col]=msg.PathL0x[col]
        XPath[1][col]=msg.PathL1x[col]
        XPath[2][col]=msg.PathL2x[col]
        XPath[3][col]=msg.PathL3x[col]
        XPath[4][col]=msg.PathL4x[col]
        XPath[5][col]=msg.PathL5x[col]

        YPath[0][col]=msg.PathL0y[col]
        YPath[1][col]=msg.PathL1y[col]
        YPath[2][col]=msg.PathL2y[col]
        YPath[3][col]=msg.PathL3y[col]
        YPath[4][col]=msg.PathL4y[col]
        YPath[5][col]=msg.PathL5y[col]

        ZPath[0][col]=msg.PathL0z[col]
        ZPath[1][col]=msg.PathL1z[col]
        ZPath[2][col]=msg.PathL2z[col]
        ZPath[3][col]=msg.PathL3z[col]
        ZPath[4][col]=msg.PathL4z[col]
        ZPath[5][col]=msg.PathL5z[col]

        TurnPath[col]=msg.PathAng[col]

        dt = msg.DT
    startSetPath = 1
rosSubPATH = rospy.Subscriber("/simple_hexapod/Legs_paths", LegPath,legpath_cb)

mode = -1
startUp = -1
def modeSelect_cb(msg):
    global mode,startUp
    if(msg.data == -1):
        startUp = 0
    else:
        mode = msg.data
rosSubModeSelect = rospy.Subscriber("/mode_selected", Float32,modeSelect_cb)

pitchInput = 0.0
rollInput = 0.0
def PitchRollInput_cb(msg):
    global pitchInput,rollInput
    pitchInput = msg.x
    rollInput = msg.y
rosSubPitchRollInput = rospy.Subscriber("/RollPitch_input", Vector3,PitchRollInput_cb)

jleg = ['Th1','Th2','Th3']
leg = ['1','2','3','4','5','6']
joints = []
for l in leg:
    for j in jleg:
        z = j + '_' + l
        joints.append(z)
pub_angles = {}
for j in joints:
    pub = rospy.Publisher('/simple_hexapod/'+j+'_position_controller/command',Float64MultiArray,queue_size=1)
    pub_angles[j] = pub 

currentmillis = 0
prevmillis = millis()

curtime = 0
prevtime = 0

stepStartFlag = 0
once = 0

currentPathPoint = [3,9,3,9,3,9]
currentPathPoint_tw = [3,3]

def SetNextPathPoint(XP,YP,ZP,TurnP,d_t):
    global prevtime, stepStartFlag, currentPathPoint, currentPathPoint_tw

    if(d_t != -1000):
    
        for i in range(6):
        
            x = XP[i][currentPathPoint[i]]
            y = YP[i][currentPathPoint[i]]
            z = ZP[i][currentPathPoint[i]]
            yaww = TurnP[currentPathPoint_tw[i%2]]
        
            (theta1[i],theta2[i],theta3[i]) = IK(x,y,z,i,d_t,rollInput,0,yaww,0)
    
        if(curtime - prevtime >= d_t):
        
            prevtime = curtime
    
            for i in range(2):
                if(currentPathPoint_tw[i] >= 11):
                    currentPathPoint_tw[i] = 0
                else:
                    currentPathPoint_tw[i] = currentPathPoint_tw[i] + 1
            
            for i in range(6):
                if(currentPathPoint[i] >= 11):
                    currentPathPoint[i] = 0
                else:
                    currentPathPoint[i] = currentPathPoint[i] + 1
    else:
        # for i in range(6):
        #     x = XP[i][currentPathPoint[i]]
        #     y = YP[i][currentPathPoint[i]]
        #     z = ZP[i][currentPathPoint[i]]
        #     yaww = TurnP[currentPathPoint_tw[i%2]]
        
        #     (theta1[i],theta2[i],theta3[i]) = IK(x,y,z,i,500,0,0,yaww)
        
        for i in range(6):
            currentPathPoint_tw[i%2] = 3
            currentPathPoint[i] = 3 if (i%2 == 0) else 9
        prevtime = curtime
        stepStartFlag = 0
    
Angle = [None for i in range(18)]

def SetAngles(th1,th2,th3):

    for num in range(18):

        if (num == 0): #thata11
            Angle[num] = th1[0]

        elif (num == 1):  #thata21
            Angle[num] = th2[0]
            
        elif (num == 2):  #thata31
            Angle[num] = th3[0]

        elif (num == 3):  #thata12
            Angle[num] = th1[1]

        elif (num == 4):  #thata22
            Angle[num] = th2[1]
 
        elif (num == 5):  #thata32
            Angle[num] = th3[1]

        elif (num == 6):  #thata13
            Angle[num] = th1[2]

        elif (num == 7):  #thata23
            Angle[num] = th2[2]

        elif (num == 8):  #thata33
            Angle[num] = th3[2]

        elif (num == 9):  #thata14
            Angle[num] = th1[3]

        elif (num == 10): #thata24
            Angle[num] = th2[3]

        elif (num == 11): #thata34
            Angle[num] = th3[3]

        elif (num == 12): #thata15
            Angle[num] = th1[4]

        elif (num == 13): #thata25
            Angle[num] = th2[4]

        elif (num == 14): #thata35
            Angle[num] = th3[4]

        elif (num == 15): #thata16
            Angle[num] = th1[5]

        elif (num == 16): #thata26
            Angle[num] = th2[5]

        elif (num == 17): #thata36
            Angle[num] = th3[5]  

    for j,v,n in zip(joints,Angle,range(18)):
        pub_angles[j].publish(data=[n,v])

kinematicModeStartFlag = 0

while 1:
    currentmillis = millis()

    if(currentmillis-prevmillis >= 10):
        prevmillis = currentmillis

        if(startUp == 0):

            if once == 0:
                startUp_startTime = millis()
                once = 1

            (theta1[0],theta2[0],theta3[0]) = IK(283.71, 0.0, -140,0,0,0,0,0,0)
            (theta1[1],theta2[1],theta3[1]) = IK(141.855, -245.7, -140,1,0,0,0,0,0)
            (theta1[2],theta2[2],theta3[2]) = IK(-141.855, -245.7, -140,2,0,0,0,0,0)
            (theta1[3],theta2[3],theta3[3]) = IK(-283.71, 0.0, -140,3,0,0,0,0,0)
            (theta1[4],theta2[4],theta3[4]) = IK(-141.855, 245.7, -140,4,0,0,0,0,0)
            (theta1[5],theta2[5],theta3[5]) = IK(141.855, 245.7, -140,5,0,0,0,0,0)
            SetAngles(theta1,theta2,theta3)
            if(currentmillis - startUp_startTime >= 5000): 
                startUp = 1

        if(mode == 0 and startUp == 1):
            stepStartFlag = 0
            
            if(kinematicModeStartFlag == 0):

                kinematicModeStartTimer = millis()
                kinematicModeStartFlag = 1

            if(currentmillis - kinematicModeStartTimer <= 1000):

                (T_theta1[0],T_theta2[0],T_theta3[0]) = IK(283.71+tx, 0.0+ty, -140-tz,0,1000,troll,tpitch,tyaw,0)
                (T_theta1[1],T_theta2[1],T_theta3[1]) = IK(141.855+tx, -245.7+ty, -140-tz,1,1000,troll,tpitch,-tyaw,0)
                (T_theta1[2],T_theta2[2],T_theta3[2]) = IK(-141.855+tx, -245.7+ty, -140-tz,2,1000,troll,tpitch,tyaw,0)
                (T_theta1[3],T_theta2[3],T_theta3[3]) = IK(-283.71+tx, 0.0+ty, -140-tz,3,1000,troll,tpitch,-tyaw,0)
                (T_theta1[4],T_theta2[4],T_theta3[4]) = IK(-141.855+tx, 245.7+ty, -140-tz,4,1000,troll,tpitch,tyaw,0)
                (T_theta1[5],T_theta2[5],T_theta3[5]) = IK(141.855+tx, 245.7+ty, -140-tz,5,1000,troll,tpitch,-tyaw,0)
                SetAngles(T_theta1,T_theta2,T_theta3)

            else:

                (T_theta1[0],T_theta2[0],T_theta3[0]) = IK(283.71+tx, 0.0+ty, -140-tz,0,0,troll,tpitch,tyaw,0)
                (T_theta1[1],T_theta2[1],T_theta3[1]) = IK(141.855+tx, -245.7+ty, -140-tz,1,0,troll,tpitch,-tyaw,0)
                (T_theta1[2],T_theta2[2],T_theta3[2]) = IK(-141.855+tx, -245.7+ty, -140-tz,2,0,troll,tpitch,tyaw,0)
                (T_theta1[3],T_theta2[3],T_theta3[3]) = IK(-283.71+tx, 0.0+ty, -140-tz,3,0,troll,tpitch,-tyaw,0)
                (T_theta1[4],T_theta2[4],T_theta3[4]) = IK(-141.855+tx, 245.7+ty, -140-tz,4,0,troll,tpitch,tyaw,0)
                (T_theta1[5],T_theta2[5],T_theta3[5]) = IK(141.855+tx, 245.7+ty, -140-tz,5,0,troll,tpitch,-tyaw,0)
                SetAngles(T_theta1,T_theta2,T_theta3)

        elif(mode == 1 and startUp == 1):
            stepStartFlag = 0
            kinematicModeStartFlag = 0
            
            A_theta1[0] = 0
            A_theta2[0] = 0
            A_theta3[0] = 0

            (px,py,pz) = FK03_inbody(A_theta1[0],A_theta2[0],A_theta3[0],0)
            (A_theta1[0],A_theta2[0],A_theta3[0]) = IK(px,py,pz,0,0,0,0,0,0)

            (px,py,pz) = FK03_inbody(A_theta1[1],A_theta2[1],A_theta3[1],1)
            (A_theta1[1],A_theta2[1],A_theta3[1]) = IK(px,py,pz,1,0,0,0,0,0)

            (px,py,pz) = FK03_inbody(A_theta1[2],A_theta2[2],A_theta3[2],2)
            (A_theta1[2],A_theta2[2],A_theta3[2]) = IK(px,py,pz,2,0,0,0,0,0)

            (px,py,pz) = FK03_inbody(A_theta1[3],A_theta2[3],A_theta3[3],3)
            (A_theta1[3],A_theta2[3],A_theta3[3]) = IK(px,py,pz,3,0,0,0,0,0)

            (px,py,pz) = FK03_inbody(A_theta1[4],A_theta2[4],A_theta3[4],4)
            (A_theta1[4],A_theta2[4],A_theta3[4]) = IK(px,py,pz,4,0,0,0,0,0)

            (px,py,pz) = FK03_inbody(A_theta1[5],A_theta2[5],A_theta3[5],5)
            (A_theta1[5],A_theta2[5],A_theta3[5]) = IK(px,py,pz,5,0,0,0,0,0)
            
            SetAngles(A_theta1,A_theta2,A_theta3)
            #mode = 2

        elif(mode == 2 and startSetPath == 1 and startUp == 1):
            kinematicModeStartFlag = 0
            
            if(stepStartFlag == 0):

                stepStartTimer = millis()
                stepStartFlag = 1

            if(currentmillis - stepStartTimer <= 1000):

                SetNextPathPoint(XPath,YPath,ZPath,TurnPath,1000)
                SetAngles(theta1,theta2,theta3)
                prevtime = millis()

            else:

                curtime = millis()
                SetNextPathPoint(XPath,YPath,ZPath,TurnPath,dt)
                SetAngles(theta1,theta2,theta3)
      
    