function [dydt] = odeFuncCircleWind(t,y,c)

dydt = zeros(4,1);

r = 100;
lambda = 0.1;
uav_x = y(1); uav_y = y(2);
origin_x = 0; origin_y = 0;
theta = atan2((uav_y - origin_y),(uav_x - origin_x));
x_t = r*cos(theta + lambda);
y_t = r*sin(theta + lambda);

si_p = atan2((y_t - uav_y),(x_t - uav_x));
si_w = 0;

v = 25;
vw = c*v; 
v_wx = 0.2*v*cos(si_w);
v_wy = 0.2*v*sin(si_w);

db = 4;
q2 = 1;

d = y(4);
q1 = sqrt(abs(db/(db - d)));
si = y(3);
vd = v*sin(si - si_p) + vw*sin(si_w - si_p); % d_dot
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);
si_dot = u/v;

dydt(1) = v*cos(si) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d

