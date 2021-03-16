function [xp,yp] = PP3_W(x,y,tf,PathSize)

    %X-path
    a0x = x(1);
    a1x = 0;
    a2x = 0;
    a3x = (2/tf^3)*(32*(x(2)-x(1))-11*(x(3)-x(1)));
    a4x = -(3/tf^4)*(64*(x(2)-x(1))-27*(x(3)-x(1)));
    a5x = (3/tf^5)*(64*(x(2)-x(1))-30*(x(3)-x(1)));
    a6x = -(32/tf^6)*(2*(x(2)-x(1))-(x(3)-x(1)));
    Xp = @(t) a0x + a1x*t + a2x*t^2 + a3x*t^3 + a4x*t^4 + a5x*t^5 + a6x*t^6;

    %Y-path
    a0y = y(1);
    a1y = 0;
    a2y = 0;
    a3y = (2/tf^3)*(32*(y(2)-y(1))-11*(y(3)-y(1)));
    a4y = -(3/tf^4)*(64*(y(2)-y(1))-27*(y(3)-y(1)));
    a5y = (3/tf^5)*(64*(y(2)-y(1))-30*(y(3)-y(1)));
    a6y = -(32/tf^6)*(2*(y(2)-y(1))-(y(3)-y(1)));
    Yp = @(t) a0y + a1y*t + a2y*t^2 + a3y*t^3 + a4y*t^4 + a5y*t^5 + a6y*t^6;

    dt = tf/(PathSize-1);
    t = 0;

    xp = zeros(1,PathSize*2-1);
    yp = zeros(1,PathSize*2-1);

    %Create path points
    for i = 1:PathSize
        xp(i) = Xp(t);
        yp(i) = Yp(t);
        t = t + dt;
    end

    %Points for support phase
    for i = (PathSize+1):(PathSize*2-1)
        xp(i) = xp(7);
        yp(i) = yp(7);
    end

end


