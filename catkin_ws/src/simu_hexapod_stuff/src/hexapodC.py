import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from my_message.msg import thetaMessage, LegPath, PathVar_n_cmdVel

class HexapodC:

    x = 0.0; y = 0.0; t = 0.0
    BH = 140.0; Ss = 100.0; Sh = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]; Fh = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; Rd = 283.71; p=0.0; r=0.0

    def __init__(self, ns='/simple_hexapod/'):
        self.ns = ns
        self.joints = None
        self.angles = None
        self.sendpathflag = None
        
        jleg = ['Th1','Th2','Th3']
        leg = ['1','2','3','4','5','6']
        joints = []
        for j in jleg:
            for l in leg:
                z = j + '_' + l
                joints.append(z)

        self.joints = joints

        rospy.loginfo('Creating joint command publisher')
        pub = rospy.Publisher(ns+'Th'+'_position_controller/command',thetaMessage,queue_size=1)
        self.pub_angles = pub 

        self.pub_path = rospy.Publisher(ns + 'Legs_paths', LegPath, queue_size=1)
        self.pub_pathDot = rospy.Publisher(ns + 'Legs_pathsDot', LegPath, queue_size=1)

        rospy.sleep(1)

        self._pub_cmd_vel_path_var = rospy.Publisher(ns + 'cmd_vel_path_var', PathVar_n_cmdVel, queue_size=1)
        

    def set_walk_velocity(self, x, y, t):
        HexapodC.x = x 
        HexapodC.y = y
        HexapodC.t = t

        msg = PathVar_n_cmdVel()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = t
        msg.path_var.BH = HexapodC.BH
        msg.path_var.Ss = HexapodC.Ss
        msg.path_var.Sh = HexapodC.Sh
        msg.path_var.Fh = HexapodC.Fh
        msg.path_var.Rd = HexapodC.Rd
        msg.path_var.p = HexapodC.p
        msg.path_var.r = HexapodC.r
        self._pub_cmd_vel_path_var.publish(msg)

    def set_path_var(self,BH = 140.0, Ss = 100.0, Sh = [50.0, 50.0, 50.0, 50.0, 50.0, 50.0], Fh = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Rd = 283.71, p=0.0, r=0.0):
        HexapodC.BH = BH
        HexapodC.Ss = Ss
        HexapodC.Sh = Sh
        HexapodC.Fh = Fh
        HexapodC.Rd = Rd
        HexapodC.p = p
        HexapodC.r = r

        msg = PathVar_n_cmdVel()
        msg.linear.x = HexapodC.x
        msg.linear.y = HexapodC.y
        msg.angular.z = HexapodC.t
        msg.path_var.BH = BH
        msg.path_var.Ss = Ss
        msg.path_var.Sh = Sh
        msg.path_var.Fh = Fh
        msg.path_var.Rd = Rd
        msg.path_var.p = p
        msg.path_var.r = r
        self._pub_cmd_vel_path_var.publish(msg)

    def get_angles(self): #Needs a subscriber
        if self.joints is None:
            return None
        if self.angles is None:
            return None
        return dict(zip(self.joints, self.angles))

    def set_angles(self,th1,th2,th3):
        msg = thetaMessage()

        msg.th1_1 = th1[0]
        msg.th1_2 = th1[1]
        msg.th1_3 = th1[2]
        msg.th1_4 = th1[3]
        msg.th1_5 = th1[4]
        msg.th1_6 = th1[5]

        msg.th2_1 = th2[0]
        msg.th2_2 = th2[1]
        msg.th2_3 = th2[2]
        msg.th2_4 = th2[3]
        msg.th2_5 = th2[4]
        msg.th2_6 = th2[5]

        msg.th3_1 = th3[0]
        msg.th3_2 = th3[1]
        msg.th3_3 = th3[2]
        msg.th3_4 = th3[3]
        msg.th3_5 = th3[4]
        msg.th3_6 = th3[5]

        self.pub_angles.publish(msg)

    def set_path(self,xpath,ypath,zpath,turnpath,xpathDot,ypathDot,zpathDot,turnpathDot,Dt):
        if self.sendpathflag == 1:
            msgg = LegPath(); msggDot = LegPath()

            msgg.PathL0x = list(xpath[0,]); msggDot.PathL0x = list(xpathDot[0,])
            msgg.PathL1x = list(xpath[1,]); msggDot.PathL1x = list(xpathDot[1,])
            msgg.PathL2x = list(xpath[2,]); msggDot.PathL2x = list(xpathDot[2,])
            msgg.PathL3x = list(xpath[3,]); msggDot.PathL3x = list(xpathDot[3,])
            msgg.PathL4x = list(xpath[4,]); msggDot.PathL4x = list(xpathDot[4,])
            msgg.PathL5x = list(xpath[5,]); msggDot.PathL5x = list(xpathDot[5,])

            msgg.PathL0y = list(ypath[0,]); msggDot.PathL0y = list(ypathDot[0,])
            msgg.PathL1y = list(ypath[1,]); msggDot.PathL1y = list(ypathDot[1,])
            msgg.PathL2y = list(ypath[2,]); msggDot.PathL2y = list(ypathDot[2,])
            msgg.PathL3y = list(ypath[3,]); msggDot.PathL3y = list(ypathDot[3,])
            msgg.PathL4y = list(ypath[4,]); msggDot.PathL4y = list(ypathDot[4,])
            msgg.PathL5y = list(ypath[5,]); msggDot.PathL5y = list(ypathDot[5,])
    
            msgg.PathL0z = list(zpath[0,]); msggDot.PathL0z = list(zpathDot[0,])
            msgg.PathL1z = list(zpath[1,]); msggDot.PathL1z = list(zpathDot[1,])
            msgg.PathL2z = list(zpath[2,]); msggDot.PathL2z = list(zpathDot[2,])
            msgg.PathL3z = list(zpath[3,]); msggDot.PathL3z = list(zpathDot[3,])
            msgg.PathL4z = list(zpath[4,]); msggDot.PathL4z = list(zpathDot[4,])
            msgg.PathL5z = list(zpath[5,]); msggDot.PathL5z = list(zpathDot[5,])

            msgg.PathAng = list(turnpath[0,]); msggDot.PathAng = list(turnpathDot[0,])

            msgg.DT = Dt

            self.pub_path.publish(msgg)
            self.pub_pathDot.publish(msggDot)
            self.sendpathflag = 0
