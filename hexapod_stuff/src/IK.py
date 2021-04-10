from interpo import Interpolation
import math 

inX1 = Interpolation()
inY1 = Interpolation()
inZ1 = Interpolation()
inX2 = Interpolation()
inY2 = Interpolation()
inZ2 = Interpolation()
inX3 = Interpolation()
inY3 = Interpolation()
inZ3 = Interpolation()
inX4 = Interpolation()
inY4 = Interpolation()
inZ4 = Interpolation()
inX5 = Interpolation()
inY5 = Interpolation()
inZ5 = Interpolation()
inX6 = Interpolation()
inY6 = Interpolation()
inZ6 = Interpolation()

def IK(x,y,z,leg,dt):

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
    if leg == 1:
        xp = inX2.go(x,dt)
        yp = inY2.go(y,dt)
        zp = inZ2.go(z,dt)
    if leg == 2:
        xp = inX3.go(x,dt)
        yp = inY3.go(y,dt)
        zp = inZ3.go(z,dt)
    if leg == 3:
        xp = inX4.go(x,dt)
        yp = inY4.go(y,dt)
        zp = inZ4.go(z,dt)
    if leg == 4:
        xp = inX5.go(x,dt)
        yp = inY5.go(y,dt)
        zp = inZ5.go(z,dt)
    if leg == 5:
        xp = inX6.go(x,dt)
        yp = inY6.go(y,dt)
        zp = inZ6.go(z,dt)

    #Transform body to leg coordinate
    xL = xp*math.cos(rot[leg]) - yp*math.sin(rot[leg]) - B
    yL = xp*math.sin(rot[leg]) + yp*math.cos(rot[leg])
    zL = zp

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
