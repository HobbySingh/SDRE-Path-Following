function [dydt] = odeFuncSLine(t,y)

global u_arr;

dydt = zeros(4,1);

w1_x = 0; w1_y = 0;
w2_x = 300; w2_y = 300;
si_p = atan2((w2_y - w1_y),(w2_x - w1_x));
v = 25;
db = 4;
q2 = 1;

d = y(4);
q1 = sqrt(abs(db/(db - d)));
si = y(3);
vd = v*sin(si - si_p); % d_dot
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);
si_dot = u/v;  
u_arr = [u_arr, u];
dydt(1) = v*cos(si); % y(1) -> uav_x
dydt(2) = v*sin(si); % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d

