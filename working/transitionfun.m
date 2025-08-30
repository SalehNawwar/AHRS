function new_x = transitionfun(x,u)
    % x = [qw qx qy qz bgx bgy bgz gxref gyref gzref mxref myref mzref]
    % u = [dt wx wy wz]
    dt=u(1);
    bg = x(5:7)';
    w = deg2rad(u(2:4)-bg);
    dq = quatnormalize([1,0.5*w*dt]);
    new_q = quatnormalize(quatmultiply(x(1:4)', dq));
    R = quat2rotm(dq)';
    
    new_x = [new_q';x(5:7);R*x(8:10);R*x(11:13)];
end
