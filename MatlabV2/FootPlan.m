function [x,y,z] = FootPlan(Rd,dir,Ss,Sh,H,rot,p,r,ba)

    dir = ba+dir - pi;
    %Ss = Ss*2
    
    dhs = Ss/2*(tan(p)*cos(dir)+tan(r)*sin(dir));
    dhe = -Ss/2*(tan(p)*cos(dir)+tan(r)*sin(dir));
    
    %Start point
    x(1) = Rd*cos(rot(1))+(Ss/2)*cos(dir);
    y(1) = -(Rd*sin(rot(1))+(Ss/2)*sin(dir));
    z(1) = -H + dhs;

    %Mid point
    x(2) = Rd*cos(rot(1));
    y(2) = -Rd*sin(rot(1));
    z(2) = -H + Sh;

    %End point
    x(3) = Rd*cos(rot(1))-(Ss/2)*cos(dir);
    y(3) = -(Rd*sin(rot(1))-(Ss/2)*sin(dir));
    z(3) = -H + dhe;
    
end

