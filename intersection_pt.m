function x_y_z_d = intersection_pt(pt,v1,v2)

syms t

% w1_x = 0; w1_y = 0; w1_z = 0;
% w2_x = 300; w2_y = 300; w2_z = 0;

% w1_x = -2; w1_y = -4; w1_z = 5;
% w2_x = 0; w2_y = 0; w2_z = 1;

% v1 = [w1_x w1_y w1_z];
% v2 = [w2_x w2_y w2_z];
% pt = [148 147 0];

a = v1 - v2;
r = v2 + t*a;

normal_vector = r - pt;

eq = dot(normal_vector,a) == 0;

t_ = solve(eq,t);

r = v2 + t_*a;

% d = sqrt((r(1) - pt(1))^2 + (r(2) - pt(2))^2 + (r(3) - pt(3))^2);
x = double(r(1)); y = double(r(2)); z = double(r(3));
x_y_z_d = [x y z];
