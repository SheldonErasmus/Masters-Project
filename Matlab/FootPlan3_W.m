function [x,y] = FootPlan3_W(Xstart,Ystart,dir,Ss,ba)
    
    Ss = Ss*2;

    %Start point
    x(1) = Xstart;
    y(1) = Ystart;

    %Mid point
    x(2) = Xstart+(Ss/2)*cos(dir+ba);
    y(2) = Ystart+(Ss/2)*sin(-dir-ba);

    %End point
    x(3) = Xstart+(Ss)*cos(dir+ba);
    y(3) = Ystart+(Ss)*sin(-dir-ba);
    
end
