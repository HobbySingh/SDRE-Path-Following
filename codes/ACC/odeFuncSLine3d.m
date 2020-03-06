function [dydt] = odeFuncSLine3d(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,c)

global control_effort;
global q1_z_arr;
% global dz_arr;
global si_z_arr;
dydt = zeros(7,1);

v = 25;

% c = 0.2;
si_w = 2.3562; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);

db = 4;

Rmin = 75;

%% XY Controller 

% Computing the position error
% Distance of point to line (UAV Position - Desired path)

pt = [y(1) y(2) 0]; % UAV Positon vector
a1 = [w1_x w1_y 0]; % Waypoint 1 Vector
a2 = [w2_x w2_y 0]; % Waypoint 2 Vector

tmp = (y(1) - w1_x)*(w2_y - w1_y) - (y(2) - w1_y)*(w2_x - w1_x);
% To check whether the point is left or right of the desired path
if(tmp < 0)
    d_xy = point_to_line(pt,a1,a2);
else
    d_xy = -point_to_line(pt,a1,a2);
end    
% d = y(4) 

% LQR Formualtion

% -------------- Old Gain ------------------
% db = 4;
% q1 = sqrt(abs(db/(db - d_xy)));
q2 = 0.01;
% -------------- Exponential Gain -------------
k = 2;
q1 = sqrt(exp(k*(abs(d_xy))));

si = y(3);
si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
vd = v*sin(si - si_p) +  vw*sin(si_w - si_p); % d_dot
% q1_d = q1*d
% d_do = sqrt(2*q1 + q2^2)*vd
% pause(0.01)
u = -(q1*d_xy + sqrt(2*q1 + q2^2)*vd);

% Constraining the control input
if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end

% Finally heading angle
si_dot = u/v;

%% Z Controller

% Computing position error in z 

pt = [y(1) y(2) y(5)]; % UAV Vector 
a1 = [w1_x w1_y w1_z]; % Waypoint 1 Vector
a2 = [w2_x w2_y w2_z]; % Waypoint 2 Vector

if(w1_z < w2_z)
    line_vect = (a2 - a1)/norm(a2 - a1);
    pt_vect = (pt - a1)/norm(pt - a1);
else
    line_vect = (a1 - a2)/norm(a1 - a2);
    pt_vect = (pt - a2)/norm(pt - a2);
end

% fprintf("point above or below) : %f \n",pt_vect(3) - line_vect(3));
% dz1 = y(7);

% To check whether the point is above or below of the desired path
% Taking zcopoinent of resultant vectors

d = point_to_line(pt,a1,a2); % Error in Z

dz = sqrt(abs(d^2 - d_xy^2))
if(pt_vect(3) - line_vect(3) < 0)
    dz = -dz; % Error in Z
else
    dz = dz;
end    
% dz_arr = [dz_arr,dz];

% LQR Formulation

% ------------ OLD GAIN ----------------
% db = 4;
% q1_z = sqrt(abs(db/(db - dz)));
% ------------ Exponential Gain -----------
k = 2;
q1_z = sqrt(exp(k*(abs(dz))))

q2 = 0.01;
q1_z_arr = [q1_z_arr,q1_z];

si_z = y(6);

v1 = [(w2_x - w1_x) (w2_y - w1_y) (w2_z - w1_z)]; % direction vector b/w two waypoints
v2 = [0 0 1]; % direction vector normal to xy plane

si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2))));
vd_z = v*sin(si_z - si_z_p)

uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z);

q1_z*dz
sqrt(2*q1_z + q2^2)*vd_z

Rminz = 75;
if(abs(uz) > (v^2)/Rminz)
    if (uz > 0)
        uz = (v^2)/Rminz;
    else
        uz = -(v^2)/Rminz;
    end
end

control_effort = [control_effort,uz];

si_z_dot = uz/v;
si_z_arr = [si_z_arr,si_z];

dydt(1) = v*cos(si)*cos(si_z) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si)*cos(si_z) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = v*sin(si_z); %y(5) -> uav_z
dydt(6) = si_z_dot; % y(5) -> si_z
dydt(7) = vd_z; %y(7) -> dz