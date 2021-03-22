function [Px,Py,Pz] = FK03(theta1,theta2,theta3)

    % degrees to radians
    d2r = pi/180;

    % Link lengths as defined in Figure 5.3
    L1x = 53.17;
    L1z = 8;
    L2  = 101.88;
    L3  = 149.16;

    % Transformation from servo angles to end effector positions as given in
    % Equation 5.30
    C1  = cos(theta1);
    S1  = sin(theta1);
    C2  = cos(theta2);
    S2  = sin(theta2);
    C23 = cos(theta2 + theta3);
    S23 = sin(theta2 + theta3);

    T03 = [C1*C23, -C1*S23,  S1, C1*(L1x + L3*C23 + L2*C2);...
            S1*C23, -S1*S23, -C1, S1*(L1x + L3*C23 + L2*C2);...
            S23,     C23,   0, L1z + L3*S23 + L2*S2;...
            0,       0,   0, 1];
 
Px = T03(1,4);Py = T03(2,4);Pz = T03(3,4);

