function [Px2,Py2,Pz2] = FK02(theta1,theta2)

    % Link lengths as defined in Figure 5.3
    L1x = 53.17;
    L1z = 8;
    L2  = 101.88;

    
    % Transformation from servo angles to end effector positions as given in
    % Equation 5.30
    C1  = cos(theta1);
    S1  = sin(theta1);
    C2  = cos(theta2);
    S2  = sin(theta2);
    
    H01 = [C1,  0,  S1, C1*L1x;...
           S1,  0, -C1, S1*L1x;...
            0,  1,   0,    L1z;...
            0,  0,   0,     1];
   
    H12 = [C2, -S2,  0, L2*C2;...
           S2,  C2,  0, L2*S2;...
            0,   0,  1,     0;...
            0,   0,  0,     1];  
    
    H02 = H01*H12;
 
Px2 = H02(1,4);Py2 = H02(2,4);Pz2 = H02(3,4);

