function [dydt] = odeFuncMovingCircle3d2(t,y,r,center_x,center_y,speedx,speedz,c)

dydt = zeros(11,1);

if(0 < t(end) && t(end) < 50)
    speedz = 1;
end
if(50 < t(end) && t(end) < 125)
    speedz = 0;
end
if(125 < t(end) && t(end) < 175)
    speedz = -1;
end


v = 25;
% c = 0.2;
si_w = 3.14; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);
si = y(3);
si_z = y(7);
x_dot = v*cos(si) + v_wx;
y_dot = v*sin(si) + v_wy;

Rmin = 75;

igain_xy = 0;%0.0001;
igain_z = 0.001;
i_xy = y(9);
i_z = y(10);

% r = 100;
lambda = 0.1;
uav_x = y(1); uav_y = y(2); uav_z = y(6);
center_x_moving = y(5);

curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

y_ = uav_y - center_y;
x_ = uav_x - center_x_moving;
y_by_x = y_/x_;
y_by_x_dot = (y_dot*x_ - y_*(x_dot - speedx))/(x_)^2; 
theta = atan2((y_),(x_));

theta_dot = (y_by_x_dot)*(1/(1 + (y_by_x)^2));

x_t = center_x_moving + r*cos(theta + lambda);
y_t = center_y + r*sin(theta + lambda);
% si_p = atan2((y_t - uav_y),(x_t - uav_x));
si_p = theta + 1.57;
si_p_dot = theta_dot;

db = 4;
q2 = 4;

d1 = y(4);
d = r - curr_r;
% q1 = sqrt(abs(db/(db - d)));
k = 1;
q1 = sqrt(exp(k*(abs(d))));
si = y(3);

v_x = v*cos(si)*cos(si_z) + vw*cos(si_w);
v_y = v*sin(si)*cos(si_z) + vw*sin(si_w);
course_angle = atan2(v_y,v_x);

vd = v*sin(course_angle - si_p); % d_dot
% vd = v*sin(si - si_p) + vw*sin(si_w - si_p); % d_dot

u = -(q1*d + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);

if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end
si_dot = u/v;
% si_p_dot = sqrt(v_x^2 + v_y^2)/r;
si_dot = (u + v*si_p_dot)/v;

% z_t = 2*(theta+lambda);
z_t = y(9);
v1 = [(x_t - uav_x) (y_t - uav_y) (z_t - uav_z)];
v2 = [0 0 1];
dz1 = y(8);
dz = (uav_z - z_t);
% q1_z = sqrt(abs(db/(db - dz)));
k = 0.01;
q1_z = sqrt(exp(k*(abs(dz))));
si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2)))); 
vd_z = v*sin(si_z - si_z_p);
uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z +  + igain_z*i_z);
Rminz = 75;
if(abs(uz) > (v^2)/Rminz)
    if (uz > 0)
        uz = (v^2)/Rminz;
    else
        uz = -(v^2)/Rminz;
    end
end
si_z_dot = uz/(v);

dydt(1) = v*cos(si)*cos(si_z) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si)*cos(si_z) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = speedx;
dydt(6) = v*sin(si_z);
dydt(7) = si_z_dot; % y(7) -> si_z
dydt(8) = vd_z; %y(7) -> dz
dydt(9) = speedz;
dydt(10) = d; %y(7) -> i_dxy
dydt(11) = dz; %y(7) -> i_dz
