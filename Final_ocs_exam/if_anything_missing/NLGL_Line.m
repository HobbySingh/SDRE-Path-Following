function [dydt] = NLGL_Line(t,y,w1_x,w1_y,w2_x,w2_y)

dydt = zeros(3,1);

uav_x = y(1); uav_y = y(2);si = y(3);
% w1_x = 0; w1_y = 0;
% w2_x = 600; w2_y = 600;
L = 10;

% [x_t , y_t] = tmp(uav_x,uav_y,w1_x,w1_y,w2_x,w2_y,L);
[x_t , y_t] = carrot_chase(uav_x,uav_y,w1_x,w1_y,w2_x,w2_y);

si_w = 2.3562;
% si_p = atan2((w2_y - w1_y),(w2_x - w1_x));
v = 25;
vw = 0.2*v; 
v_wx = vw*cos(si_w);
v_wy = vw*sin(si_w);
v_x = v*cos(si);
v_y = v*sin(si);
vg_x = v_x + v_wx;
vg_y = v_y + v_wy;
Vg = sqrt(vg_x^2 + vg_y^2);
% xi= si+atan2(vg_y,vg_x);

si_p = atan2((y_t - (uav_y)),(x_t - uav_x));
eta = 1*(si_p - (si));
u = 2*((v^2)/L)*(sin(eta) );
u = 2*((v*cos(eta)*v*sin(eta))/L);

si_dot = u/v;

dydt(1) = vg_x;% + v_wx; % y(1) -> uav_x
dydt(2) = vg_y;% + v_wy; % y(2) -> uav_y
dydt(3) = si_dot; % y(3) -> si
