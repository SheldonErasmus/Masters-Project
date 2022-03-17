from math import atan2, asin, copysign, pi

def ToEulerAng(x,y,z,w):

    #roll
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x**2 + y**2)
    roll = atan2(sinr_cosp, cosr_cosp)

    #pitch
    sinp = 2*(w*y - z*x)
    if abs(sinp) >= 1:
        pitch = copysign(pi / 2, sinp) #use 90 degrees if out of range
    else:
        pitch = asin(sinp)

    #yaw
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = atan2(siny_cosp, cosy_cosp)

    return roll,pitch,yaw