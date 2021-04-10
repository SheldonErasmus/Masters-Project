from math import cos, sin
from numpy import array

def FK01(theta1):

    #Link lengths as defined in Figure 5.3
    L1x = 53.17
    L1z = 8

    #Transformation from servo angles to end effector positions as given in
    #Equation 5.30
    C1  = cos(theta1)
    S1  = sin(theta1)

    H01 = array([[C1,  0,  S1, C1*L1x],[S1,  0, -C1, S1*L1x],[0,  1,   0,    L1z],[0,  0,   0,     1]])
 
    Px1 = H01[0,3]
    Py1 = H01[1,3]
    Pz1 = H01[2,3]

    return Px1, Py1, Pz1

def FK02(theta1,theta2):

    #Link lengths as defined in Figure 5.3
    L1x = 53.17
    L1z = 8
    L2  = 101.88

    
    # Transformation from servo angles to end effector positions as given in
    # Equation 5.30
    C1  = cos(theta1)
    S1  = sin(theta1)
    C2  = cos(theta2)
    S2  = sin(theta2)
    
    H01 = array([[C1,  0,  S1, C1*L1x],
                [S1,  0, -C1, S1*L1x],
                [0,  1,   0,    L1z],
                [0,  0,   0,     1]])
   
    H12 = array([[C2, -S2,  0, L2*C2],
                [S2,  C2,  0, L2*S2],
                [0,   0,  1,     0],
                [0,   0,  0,     1]])  
    
    H02 = H01@H12
 
    Px2 = H02[0,3]
    Py2 = H02[1,3]
    Pz2 = H02[2,3]

    return Px2, Py2, Pz2

def FK03(theta1,theta2,theta3):

    # Link lengths as defined in Figure 5.3
    L1x = 53.17
    L1z = 8
    L2  = 101.88
    L3  = 149.16

    # Transformation from servo angles to end effector positions as given in
    # Equation 5.30
    C1  = cos(theta1)
    S1  = sin(theta1)
    C2  = cos(theta2)
    S2  = sin(theta2)
    C23 = cos(theta2 + theta3)
    S23 = sin(theta2 + theta3)

    T03 = array([[C1*C23, -C1*S23,  S1, C1*(L1x + L3*C23 + L2*C2)],
                [S1*C23, -S1*S23, -C1, S1*(L1x + L3*C23 + L2*C2)],
                [S23,     C23,   0, L1z + L3*S23 + L2*S2],
                [0,       0,   0, 1]])
 
    Px = T03[0,3]
    Py = T03[1,3]
    Pz = T03[2,3]

    return Px, Py, Pz