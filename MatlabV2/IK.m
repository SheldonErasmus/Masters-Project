function [theta1,theta2,theta3] = IK(xp,yp,zp,leg)
    
    L1x = 53.17;
    L1z = 8;
    L2  = 101.88;
    L3  = 149.16;
    B = 125.54;
    rot = [0*pi/180, 60*pi/180, 120*pi/180, 180*pi/180, 240*pi/180, 300*pi/180];
    
    %Transform body to leg coordinate
    xL = xp*cos(rot(leg)) - yp*sin(rot(leg)) - B;
    yL = xp*sin(rot(leg)) + yp*cos(rot(leg));
    zL = zp;


    theta1 = atan2(yL,xL);

    C1  = cos(theta1);
    S1  = sin(theta1);

    C3_temp = ( ( xL - L1x*C1 )^2 + ( yL - L1x*S1 )^2 + ( zL - L1z )^2 - L2^2 - L3^2 )/( 2*L2*L3 );
    
    if C3_temp>1
        C3 = 1;
    elseif C3_temp<-1
        C3 = -1;
    else
        C3 = C3_temp;
    end
    
    
    S3 = -sqrt((1 - C3^2));

    theta2 = atan2( zL-L1z, sqrt( (xL-L1x*C1)^2 + (yL - L1x*S1)^2 ) ) - atan2(L3*S3, L2 + L3*C3);

    theta3 = atan2(S3, C3);
    
end

