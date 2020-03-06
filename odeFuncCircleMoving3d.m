function [dydt] = odeFuncCircleMoving3d(t,y,r,center_x,center_y,speedx,speedz)

dydt = zeros(9,1);

desired_z = y(9);
lambda = 0.1;
uav_x = y(1); uav_y = y(2); uav_z = y(5);
center_x_moving = y(8);

curr_r = sqrt((uav_x - center_x_moving)^2 + (uav_y - center_y)^2);

theta = atan2((uav_y - center_y),(uav_x - center_x_moving));

x_t = center_x_moving + r*cos(theta + lambda);
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

v1 = [uav_x uav_y uav_z];
z_t = y(9);
% z_t = 2*(theta+lambda);
v2 = [x_t y_t z_t];
dz = y(7);
q1_z = sqrt(abs(db/(db - dz)));
si_z = y(6);
si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2)))); 
vd_z = v*sin(si_z - si_z_p);
uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z);
si_z_dot = uz/v;

dydt(1) = v*cos(si)*cos(si_z); % y(1) -> uav_x
dydt(2) = v*sin(si)*cos(si_z); % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = v*sin(si_z);% y(3) -> uav_z
dydt(6) = si_z_dot; % y(5) -> si_z
dydt(7) = vd_z; %y(7) -> dz
dydt(8) = speedx;
dydt(9) = speedz;