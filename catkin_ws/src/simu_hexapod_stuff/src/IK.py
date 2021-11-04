from interpo import Interpolation
import math 

inX1 = Interpolation(429.75)
inY1 = Interpolation(0.0)
inZ1 = Interpolation(8.0)
inYaw1 = Interpolation(0.0)

inX2 = Interpolation(214.875)
inY2 = Interpolation(-372.174)
inZ2 = Interpolation(8.0)
inYaw2 = Interpolation(0.0)

inX3 = Interpolation(-214.875)
inY3 = Interpolation(-372.174)
inZ3 = Interpolation(8.0)
inYaw3 = Interpolation(0.0)

inX4 = Interpolation(-429.75)
inY4 = Interpolation(0.0)
inZ4 = Interpolation(8.0)
inYaw4 = Interpolation(0.0)

inX5 = Interpolation(-214.875)
inY5 = Interpolation(372.174)
inZ5 = Interpolation(8.0)
inYaw5 = Interpolation(0.0)

inX6 = Interpolation(214.875)
inY6 = Interpolation(372.174)
inZ6 = Interpolation(8.0)
inYaw6 = Interpolation(0.0)



def IK(x,y,z,leg,dt,roll=0.0,pitch=0.0,yaw=0.0,inEn=1):

    L1x = 53.17
    L1z = 8
    L2  = 101.88
    L3  = 149.16
    B = 125.54
    rot = [0*math.pi/180, 60*math.pi/180, 120*math.pi/180, 180*math.pi/180, 240*math.pi/180, 300*math.pi/180]


    if leg == 0:
        xp = inX1.go(x,dt)
        yp = inY1.go(y,dt)
        zp = inZ1.go(z,dt)
        yawp = inYaw1.go(yaw,dt)
    if leg == 1:
        xp = inX2.go(x,dt)
        yp = inY2.go(y,dt)
        zp = inZ2.go(z,dt)
        yawp = inYaw2.go(-yaw,dt)
    if leg == 2:
        xp = inX3.go(x,dt)
        yp = inY3.go(y,dt)
        zp = inZ3.go(z,dt)
        yawp = inYaw3.go(yaw,dt)
    if leg == 3:
        xp = inX4.go(x,dt)
        yp = inY4.go(y,dt)
        zp = inZ4.go(z,dt)
        yawp = inYaw4.go(-yaw,dt)
    if leg == 4:
        xp = inX5.go(x,dt)
        yp = inY5.go(y,dt)
        zp = inZ5.go(z,dt)
        yawp = inYaw5.go(yaw,dt)
    if leg == 5:
        xp = inX6.go(x,dt)
        yp = inY6.go(y,dt)
        zp = inZ6.go(z,dt)
        yawp = inYaw6.go(-yaw,dt)



    # ** YAW CALCS **

    #degree to radians
    yaw_rad = yawp * math.pi/180.0

    #calc existing yaw angle
    existingAngle = math.atan2(yp,xp)

    #calc radius from centre
    radius = math.sqrt(xp**2+yp**2)

    #calc demand yaw angle
    demandYaw = existingAngle + yaw_rad

    # if leg == 0: #########################################
    #     print([zp, demandYaw])

    #calc new x and y based on demand yaw
    xxp = radius*math.cos(demandYaw)
    yyp = radius*math.sin(demandYaw)

    #calc new z based on demand pitch and roll
    zzp = zp + xxp*math.tan(pitch*math.pi/180.0)+yyp*math.tan(roll*math.pi/180.0)

    #Transform body to leg coordinate
    xL = xxp*math.cos(rot[leg]) - yyp*math.sin(rot[leg]) - B
    yL = xxp*math.sin(rot[leg]) + yyp*math.cos(rot[leg])
    zL = zzp

    theta1 = math.atan2(yL,xL)

    C1  = math.cos(theta1)
    S1  = math.sin(theta1)

    C3_temp = ( ( xL - L1x*C1 )**2 + ( yL - L1x*S1 )**2 + ( zL - L1z )**2 - L2**2 - L3**2 )/( 2*L2*L3 )
    
    if C3_temp>1:
        C3 = 1
    elif C3_temp<-1:
        C3 = -1
    else:
        C3 = C3_temp
    
    S3 = -math.sqrt((1 - C3**2))

    theta2 = math.atan2( zL-L1z, math.sqrt( (xL-L1x*C1)**2 + (yL - L1x*S1)**2 ) ) - math.atan2(L3*S3, L2 + L3*C3)

    theta3 = math.atan2(S3, C3)

    return theta1, theta2, theta3
