function [dydt] = odeFuncSLine3d_lqrIntegral(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,c)

dydt = zeros(9,1);

v = 25;

si_w = 3.14; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);

Rmin = 75;

igain_xy = 0;%0.001;
igain_z = 0.1;
i_xy = y(8);
i_z = y(9);

si_z = y(6);

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

% LQR Formualtion

% -------------- Old Gain ------------------
% db = 4;
% q1 = sqrt(abs(db/(db - d_xy)));
q2 = 1;
% -------------- Exponential Gain -------------
k = 0.01;
q1 = sqrt(exp(k*(abs(d_xy))));

q3 = 0.01;
si = y(3);

% ------------- addition of beta ---------------
% course_angle = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
% wind_vel_par = v_wx*cos(course_angle) + v_wy*sin(course_angle);
% wind_vel_per = -v_wx*sin(course_angle) + v_wy*cos(course_angle);
% v_gs = sqrt((v*cos(si_z))^2 - wind_vel_per^2) + wind_vel_par
% deg2rad(((v*cos(si_z))^2 + v_gs^2 - vw^2)/2*v*cos(si_z)*v_gs);
% beta = acos(deg2rad(((v*cos(si_z))^2 + v_gs^2 - vw^2)/2*v*cos(si_z)*v_gs));
% beta = -beta*sign(wind_vel_per);
% beta = -asin((vw*sin(si_w))/v);
% beta = atan2((v*sin(si) + vw*sin(si_w)),(v*cos(si) + vw*cos(si_w)))
% si = si - beta
%------------------------------------------------
% [uav_vector_x,uav_vector_y] = pol2cart(si,v);
% [wind_vector_x,wind_vector_y] = pol2cart(si_w,vw);
% uav_vector = [uav_vector_x,uav_vector_y];
% wind_vector = [wind_vector_x,wind_vector_y];
% course_vector = uav_vector + wind_vector;
% 
% crab_angle = acos(dot(wind_vector,course_vector));
% course_angle = si + crab_angle;

v_x = v*cos(si) + vw*cos(si_w);
v_y = v*sin(si) + vw*sin(si_w);

course_angle = atan2(v_y,v_x);
si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
% vd = v*sin(si - si_p) +  vw*sin(si_w - si_p); % d_dot
vd = v*cos(si_z)*sin(course_angle - si_p) +  vw*sin(si_w - si_p); % d_dot

u = -(q1*d_xy + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);

A = [0,1,0;0,0,0;1,0,0];
B = [0;1;0];
Q = [q1,0,0;0,q2,0;0,0,q3];
R = [1];
X = [d_xy;vd;i_xy];
[K] = lqr(A,B,Q,R);

u = -K*X;
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
% Taking zcopoinent of resultant vectors

d = point_to_line(pt,a1,a2); % Error in Z

dz = sqrt(abs(d^2 - d_xy^2));
if(pt_vect(3) - line_vect(3) < 0)
    dz = -dz; % Error in Z
else
    dz = dz;
end    

% LQR Formulation

% ------------ OLD GAIN ----------------
% db = 4;
% q1_z = sqrt(abs(db/(db - dz)));
% ------------ Exponential Gain -----------
k = 0.01;
q1_z = sqrt(exp(k*(abs(dz))));

q2 = 0.1;

v1 = [(w2_x - w1_x) (w2_y - w1_y) (w2_z - w1_z)]; % direction vector b/w two waypoints
v2 = [0 0 1]; % direction vector normal to xy plane

si_z_p = asin(dot(v1,v2)/(norm(v1)*(norm(v2))));
vd_z = v*sin(si_z - si_z_p);
uz = -(q1_z*dz + sqrt(2*q1_z + q2^2)*vd_z + igain_z*i_z);

Rminz = 75;
if(abs(uz) > (v^2)/Rminz)
    if (uz > 0)
        uz = (v^2)/Rminz;
    else
        uz = -(v^2)/Rminz;
    end
end

si_z_dot = uz/v;

dydt(1) = v*cos(si)*cos(si_z) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si)*cos(si_z) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d
dydt(5) = v*sin(si_z); %y(5) -> uav_z
dydt(6) = si_z_dot; % y(5) -> si_z
dydt(7) = vd_z; %y(7) -> dz
dydt(8) = d_xy; %y(7) -> i_dxy
dydt(9) = dz; %y(7) -> i_dz