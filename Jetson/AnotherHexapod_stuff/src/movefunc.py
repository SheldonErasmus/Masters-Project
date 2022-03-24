from math import pi,sin,cos
#from IK import *
#from FK import *
import TurnPath
import WalkPath
from numpy import zeros
import time

class MoveFunc:
    def __init__(self):
        #Constant variables
        self.B = 125.54
        self.rot = [0*pi/180, 60*pi/180, 120*pi/180, 180*pi/180, 240*pi/180, 300*pi/180]

        #Global variables
        self.currentPathPoint = [3,9,3,9,3,9]; self.currentPathPoint_w = [3,9]; self.currentPathPoint_t = [0,0]; self.currentPathPoint_tw = [3,3]
        self.PosX = zeros((6,)); self.PosY = zeros((6,)); self.PosZ = zeros((6,))
        self.theta1 = zeros((6,)); self.theta2 = zeros((6,)); self.theta3 = zeros((6,)); self.OG_theta1 = zeros((6,)); self.OG_PosZ = zeros((6,))
        self.theta1_d = zeros((6,)); self.theta2_d = zeros((6,)); self.theta3_d = zeros((6,))
        self.bodyTwist = 0.0
        self.mode = ""
        self.legRotation = 0.0
        self.prevtime = 0
        self.currenttime = 0

    def Leg2body(self,x,y,rot):
        XG = y*sin(rot)+(x+self.B)*cos(rot)
        YG = y*cos(rot)-(x+self.B)*sin(rot)
        return XG,YG

    def Body2leg(self,x,y,rot):
        XL = x*cos(rot) - y*sin(rot) - self.B
        YL = x*sin(rot) + y*cos(rot)
        return XL,YL

    def makepath_walk(self,Ss,Sh,Rd,p,r,di,BH,tf = 2,pathsize = 7):
        WP = WalkPath.WalkPath(Rd,di,Ss,Sh,BH,self.rot[0],p,r)
        xp = zeros((6,7*2-2)); yp = zeros((6,7*2-2)); zp = zeros((6,7*2-2))
        xp_d = zeros((6,7*2-2)); yp_d = zeros((6,7*2-2)); zp_d = zeros((6,7*2-2))
        for i in range(6):
            WP.rot = self.rot[i]
            (xp[i,],yp[i,],zp[i,],xp_d[i,],yp_d[i,],zp_d[i,]) = WP.MakePath(tf,pathsize)
        
        return xp,yp,zp,xp_d,yp_d,zp_d

    def makepath_turn(self,AngSS,ZSS,tf = 2,pathsize = 7):
        TP = TurnPath.TurnPath(0,0,AngSS,ZSS)
        (ang,z,ang_d,z_d) = TP.MakePath(tf,pathsize)
        
        return ang,z,ang_d,z_d

    #Below this needs to be implemented on Teensy if needed    

    """def setNextPathPoint(self,xp,yp,zp,xpd,ypd,zpd,dt,yaw):
        for i in range(6):
            self.PosX[i] = xp[i,self.currentPathPoint[i]]
            self.PosY[i] = yp[i,self.currentPathPoint[i]]
            self.PosZ[i] = zp[i,self.currentPathPoint[i]]
            YAW = yaw[0,self.currentPathPoint_tw[i%2]]

            (self.theta1[i], self.theta2[i], self.theta3[i]) = IK(self.PosX[i],self.PosY[i],self.PosZ[i],i,dt,yaw = YAW)

            Xd = xpd[i,self.currentPathPoint[i]]
            Yd = ypd[i,self.currentPathPoint[i]]
            Zd = zpd[i,self.currentPathPoint[i]]

            (self.theta1_d[i], self.theta2_d[i], self.theta3_d[i]) = self.SetSpeed_pos(Xd,Yd,Zd,i)

        if self.currenttime - self.prevtime >= dt:
            self.prevtime = self.currenttime

            for i in range(2):
                if(self.currentPathPoint_tw[i] >= 11):
                    self.currentPathPoint_tw[i] = 0
                else:
                    self.currentPathPoint_tw[i] = self.currentPathPoint_tw[i] + 1

            for i in range(6):
                if(self.currentPathPoint[i] >= 11):
                    self.currentPathPoint[i] = 0
                else:
                    self.currentPathPoint[i] = self.currentPathPoint[i] + 1

    def SetPosition(self,x,y,z,dt,leg=None):
        if leg is None:
            for i in range(6):
                self.PosX[i] = x[i]
                self.PosY[i] = y[i]
                self.PosZ[i] = z[i]
                (self.theta1[i], self.theta2[i], self.theta3[i]) = IK(self.PosX[i],self.PosY[i],self.PosZ[i],i,dt)
        else:
            self.PosX[leg] = x
            self.PosY[leg] = y
            self.PosZ[leg] = z
            (self.theta1[leg], self.theta2[leg], self.theta3[leg]) = IK(self.PosX[leg],self.PosY[leg],self.PosZ[leg],leg,dt)

    # def SetTheta(self,th1,th2,th3,leg):
    #     self.theta1[leg] = th1
    #     self.theta2[leg] = th2
    #     self.theta3[leg] = th3

    #     (PosX_L, PosY_L, PosZ_L) = FK03(self.theta1[leg],self.theta2[leg],self.theta3[leg])
    #     (self.PosX[leg],self.PosY[leg]) = self.Leg2body(PosX_L,PosY_L,self.rot[leg])
    #     self.PosZ[leg] = PosZ_L

    def SetSpeed_pos(self,xd,yd,zd,leg):
        th1 = self.theta1[leg]
        th2 = self.theta2[leg]
        th3 = self.theta3[leg]

        xd_leg = xd/1000*cos(self.rot[leg]) - yd/1000*sin(self.rot[leg])
        yd_leg = xd/1000*sin(self.rot[leg]) + yd/1000*cos(self.rot[leg])
        zd_leg = zd/1000

        L1x = 53.17/1000
        L1z = 8/1000
        L2  = 101.88/1000
        L3  = 149.16/1000

        thd1 = (yd_leg*cos(th1) - xd_leg*sin(th1))/(L1x + L3*cos(th2 + th3) + L2*cos(th2))
        thd2 = (1/L2)*( zd_leg*cos(th2) - xd_leg*cos(th1)*sin(th2) - yd_leg*sin(th1)*sin(th2) + ((cos(th3))/(sin(th3)))*(zd_leg*sin(th2) + xd_leg*cos(th1)*cos(th2) + yd_leg*cos(th2)*sin(th1)) ) 
        thd3 = -(1/L2)*( zd_leg*cos(th2) - xd_leg*cos(th1)*sin(th2) - yd_leg*sin(th1)*sin(th2) + (L2 + L3*(cos(th3))/(L3*sin(th3)))*(zd_leg*sin(th2) + xd_leg*cos(th1)*cos(th2) + yd_leg*cos(th2)*sin(th1)) )

        return thd1,thd2,thd3

    def SetSpeed_th(self,th1d,th2d,th3d,leg):
        self.theta1_d[leg] = th1d
        self.theta2_d[leg] = th2d
        self.theta3_d[leg] = th3d

    def stand(self,BH,dt):
        self.SetPosition([283.71,141.855,-141.855,-283.71,-141.855,141.855],[0.0,-245.7,-245.7,0.0,245.7,245.7],[-BH,-BH,-BH,-BH,-BH,-BH],dt)
    """
    