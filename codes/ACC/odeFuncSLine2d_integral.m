function [dydt] = odeFuncSLine2d_integral(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,c)

dydt = zeros(4,1);

v = 25;

si_w = 1.57; % wind direction
vw = c*v; 
v_wx = c*v*cos(si_w);
v_wy = c*v*sin(si_w);

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

% LQR Formualtion

% -------------- Old Gain ------------------
% db = 4;
% q1 = sqrt(abs(db/(db - d_xy)));
q2 = 1;
% -------------- Exponential Gain -------------
k = 0.001;
q1 = sqrt(exp(k*(abs(d_xy))));

si = y(3);
% si_deg = rad2deg(si)

v_x = v*cos(si) + vw*cos(si_w);
v_y = v*sin(si) + vw*sin(si_w);

course_angle = atan2(v_y,v_x);
si_p = atan2((w2_y - w1_y),(w2_x - w1_x)); % si desired
% si_p_deg = rad2deg(si_p)
% vd = v*sin(si - si_p) +  vw*sin(si_w - si_p); % d_dot
vd = v*sin(course_angle - si_p); % d_dot

A = [0,1;0,0];
B = [0;(cos(si - si_p))];
Q = [q1,0;0,q2];
R = [1];
X = [d_xy;vd];
[K] = lqr(A,B,Q,R);

u = -K*X;

% p12 = q1/abs(cos(si-si_p));
% p22 = sqrt(2*p12 + q2)/(cos(si)*cos(si_p)+sin(si)*sin(si_p));
% cos(si - si_p);
% u = -(d_xy*p12*cos(si - si_p) + p22*vd*cos(si - si_p));

% u = -(q1*d_xy + sqrt(2*q1 + q2^2)*vd + igain_xy*i_xy);

if(abs(u) > (v^2)/Rmin)
    if (u > 0)
        u = (v^2)/Rmin;
    else
        u = -(v^2)/Rmin;
    end
end

% Finally heading angle
si_dot = u/v;

wp_dir = atan2((w2_y-y(2)),(w2_x-y(1)));

wp_dir_deg = wrapTo360(rad2deg(wp_dir));
si_deg = wrapTo360(rad2deg(si));
si_p_deg = wrapTo360(rad2deg(si_p));

% if abs(angdiff(deg2rad(si_deg),deg2rad(si_p_deg))) >= 1.57
%     if(wp_dir - si) > 0
%          fprintf("A\n");  
%         si_dot = abs(si_dot);
%     else
%          fprintf("B\n");
%          wp_dir_deg
%          si_deg
%          si_dot = -abs(si_dot);
%     end
% end

% if abs(abs(si) - abs(si_p)) >= 1.6573
%     if (si - si_p) > 0
%         si_dot = -abs(si_dot);
%     else
%         si_dot = abs(si_dot);
%     end
% end

dydt(1) = v*cos(si) + v_wx; % y(1) -> uav_x
dydt(2) = v*sin(si) + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
dydt(4) = vd; % y(4) -> d