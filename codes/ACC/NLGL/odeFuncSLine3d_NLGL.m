function [dydt] = odeFuncSLine3d_NLGL(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,c)

dydt = zeros(5,1);

v = 25;

si_w = 3.14; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);

Rmin = 75;

uav_x = y(1);
uav_y = y(2);
si = y(3);
uav_z = y(4);
si_z = y(5);

%% XY Controller 
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

% NLGL Formualtion
L = 70;

% [xt,yt,zt] = nlgl_vtp(uav_x,uav_y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,L);
[xt,yt,zt] = sphere_intersection(w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,uav_x,uav_y,uav_z,L);

% ------------- Course Angle-----------------
v_x = v*cos(si)*cos(si_z) + vw*cos(si_w);
v_y = v*sin(si)*cos(si_z) + vw*sin(si_w);
course_angle = atan2(v_y,v_x);

theta = atan2((yt-uav_y),(xt-uav_x));
eta = theta - course_angle;

u = 2*v^2*sin(eta)/L;

% Constraining the control input
if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end

if(nargin == 10)
    dydt = u; 
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


% To check whether the point is above or below of the desired path
% Taking zcomponent of resultant vectors

d = point_to_line(pt,a1,a2); % Error in Z

dz = sqrt(abs(d^2 - d_xy^2));
if(pt_vect(3) - line_vect(3) < 0)
    dz = -dz; % Error in Z
else
    dz = dz;
end    

% ---------- NLGL for z direction ----------------------
v1 = [(w2_x - w1_x) (w2_y - w1_y) (w2_z - w1_z)]; % direction vector b/w two waypoints
v2 = [0 0 1]; % direction vector normal to xy plane

si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2))));

desired_direction_vector = -[(uav_x - xt) (uav_y - yt) (uav_z - zt)];
theta = asin(dot(desired_direction_vector,v2)/(norm(desired_direction_vector)*(norm(v2))));

eta = theta - si_z;

uz = 2*v^2*sin(eta)/L;

Rminz = 75;
if(abs(uz) > (v^2)/Rminz)
    if (uz > 0)
        uz = (v^2)/Rminz;
    else
        uz = -(v^2)/Rminz;
    end
end

si_z_dot = uz/v;

%% State Equations
dydt(1) = v*cos(si)*cos(si_z) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si)*cos(si_z) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> uav_si (heading)
dydt(4) = v*sin(si_z); %y(5) -> uav_z
dydt(5) = si_z_dot; % y(5) -> uav_si_z (pitch)
