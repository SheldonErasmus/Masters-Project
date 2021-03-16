function [Px1,Py1,Pz1] = FK01(theta1)

    % Link lengths as defined in Figure 5.3
    L1x = 53.17;
    L1z = 8;

    % Transformation from servo angles to end effector positions as given in
    % Equation 5.30
    C1  = cos(theta1);
    S1  = sin(theta1);

    H01 = [C1,  0,  S1, C1*L1x;...
           S1,  0, -C1, S1*L1x;...
            0,  1,   0,    L1z;...
            0,  0,   0,     1];
 
Px1 = H01(1,4);Py1 = H01(2,4);Pz1 = H01(3,4);

