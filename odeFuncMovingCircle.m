function [dydt] = odeFuncMovingCircle(t,y,r,center_x,center_y,speedx,c)

dydt = zeros(5,1);

% r = 100;
lambda = 0.1;
uav_x = y(1); uav_y = y(2);
center_x_moving = y(5);
Rmin = 75;

curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

theta = atan2((uav_y - center_y),(uav_x - center_x_moving));

x_t = center_x_moving + r*cos(theta + lambda);
y_t = center_y + r*sin(theta + lambda);
% si_p = atan2((y_t - uav_y),(x_t - uav_x));
% if(theta == 0)
%     si_p = 1.57;   
% else
%     si_p = -1/(theta);
% end

si_p = theta + 1.57;

v = 25;
% c = 0.2;
si_w = 3.14; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);
db = 4;
q2 = 1;

d1 = y(4);
d = (r - curr_r);
q1 = sqrt(abs(db/(db - d)));
si = y(3);
vd = v*sin(si - si_p) + vw*sin(si_w - si_p);% d_dot
% si_dot2 = y(6);
% ad = si_dot2*v*cos(si - si_p);
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);

if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end

si_dot = u/v;

dydt(1) = v*cos(si) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = speedx;
% dydt(6) = ad;

