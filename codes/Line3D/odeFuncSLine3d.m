function [dydt] = odeFuncSLine3d(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z)

dydt = zeros(7,1);
% dydt = zeros(4,1);

% w1_x = 1; w1_y = 1; w1_z = 0;
% w2_x = 300; w2_y = 300; w2_z = 300;

v1 = [w1_x w1_y w1_z];
v2 = [w2_x w2_y w2_z];

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
dydt(5) = v*sin(si_z);
dydt(6) = si_z_dot; % y(5) -> si_z
dydt(7) = vd_z; %y(7) -> dz