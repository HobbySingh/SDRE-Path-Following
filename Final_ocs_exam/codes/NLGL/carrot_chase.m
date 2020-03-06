function [x_t,y_t] = carrot_chase(uav_x,uav_y,w1_x,w1_y,w2_x,w2_y)

delta = 10;
Ru = sqrt((w1_x - uav_x)^2+(w1_y - uav_y)^2);
theta = atan2(w2_y - uav_y, w2_x - uav_x);
theta_u = atan2(uav_y - w1_y,uav_x - w1_x);
beta = theta - theta_u;
R = sqrt(Ru^2 -(Ru*sin(beta))^2);

x_t = (R + delta)*cos(theta);
y_t = (R + delta)*sin(theta);