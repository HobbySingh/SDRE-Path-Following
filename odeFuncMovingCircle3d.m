function [dydt] = odeFuncMovingCircle3d(t,y,r,center_x,center_y,speedx,speedz,c)

dydt = zeros(9,1);
Rmin = 75;
% r = 100;
lambda = 0.1;
uav_x = y(1); uav_y = y(2); uav_z = y(6);
center_x_moving = y(5);

curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

theta = atan2((uav_y - center_y),(uav_x - center_x_moving));
x_t = center_x_moving + r*cos(theta + lambda);
y_t = center_y + r*sin(theta + lambda);
si_p = atan2((y_t - uav_y),(x_t - uav_x));
% si_p = theta + 1.57;

v = 25;
% c = 0.2;
si_w = 3.14; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);
db = 4;
q2 = 1;

d1 = y(4);
d = r - curr_r;
q1 = sqrt(abs(db/(db - d)));
si = y(3);
vd = v*sin(si - si_p) + vw*sin(si_w - si_p); % d_dot
u = -(q1*d + sqrt(2*q1 + q2^2)*vd);
if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end
si_dot = u/v;

% z_t = 2*(theta+lambda);
z_t = y(9);
v1 = [(x_t - uav_x) (y_t - uav_y) (z_t - uav_z)];
v2 = [0 0 1];
dz1 = y(8);
dz = uav_z - z_t;
q1_z = sqrt(abs(db/(db - dz)));
si_z = y(7);
si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2)))); 
vd_z = v*sin(si_z - si_z_p);
uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z);
Rminz = 75;
if(abs(uz) > (v^2)/Rminz)
    if (uz > 0)
        uz = (v^2)/Rminz;
    else
        uz = -(v^2)/Rminz;
    end
end
si_z_dot = uz/v;

dydt(1) = v*cos(si) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = speedx;
dydt(6) = v*sin(si_z);
dydt(7) = si_z_dot; % y(7) -> si_z
dydt(8) = vd_z; %y(7) -> dz
dydt(9) = speedz;

