function [xp,yp] = makepath3_W(Xstart,Ystart,dir,Ss,ba)

    tf = 2;
    PathSize = 7;

    [x_foot,y_foot] = FootPlan3_W(Xstart,Ystart,dir,Ss,ba);
    [xp,yp] = PP3_W(x_foot,y_foot,tf,PathSize);

end

