function [dydt] = odeFuncCircle(t,y,center_x,center_y)

dydt = zeros(4,1);

r = 100;
lambda = 0.1;
uav_x = y(1); uav_y = y(2);
% center_x = 25; center_y = 25;

curr_r = sqrt((uav_x - center_x)^2 + (uav_y - center_y)^2);

theta = atan2((uav_y - center_y),(uav_x - center_x));
x_t = center_x + r*cos(theta + lambda);
y_t = center_y + r*sin(theta + lambda);
si_p = atan2((y_t - uav_y),(x_t - uav_x));
v = 25;
db = 4;
q2 = 1;

d1 = y(4);
d = r - curr_r;
q1 = sqrt(abs(db/(db - d)));
si = y(3);
vd = v*sin(si - si_p); % d_dot
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);
si_dot = u/v;

dydt(1) = v*cos(si); % y(1) -> uav_x
dydt(2) = v*sin(si); % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d

