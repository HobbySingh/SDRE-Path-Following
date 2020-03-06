function [x_t,y_t] = tmp(uav_x,uav_y,w1_x,w1_y,w2_x,w2_y,L)
syms x y;

circle = (x - uav_x)^2 + (y - uav_y)^2 - L^2;
m = atan2((w2_y - w1_y),(w2_x - w1_x));
line = y - m*x ;

eqns = [circle, line];
vars = [x y];

[solx , soly] = solve(eqns, vars);

if solx(1) > solx(2)
    x_t = solx(1);
else
    x_t = solx(2);
end

if soly(1) > soly(2)
    y_t = solx(1);
else
    y_t = solx(2);
end
