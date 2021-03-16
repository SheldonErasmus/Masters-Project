function [xp,yp,zp] = makepath(Rd,dir,Ss,Sh,H,rot,p,r,ba)

    tf = 2;
    PathSize = 7;

    [x_foot,y_foot,z_foot] = FootPlan(Rd,dir,Ss,Sh,H,rot,p,r,ba);
    [xp,yp,zp] = PP(x_foot,y_foot,z_foot,tf,PathSize);

end

