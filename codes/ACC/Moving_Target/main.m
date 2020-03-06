clc; clear all;
close all;

%% Initialization of params
% waypoints = [[0, 0, 0]; [600, 0, 30]; [1200, 300, 60];[900,900,70]; [300, 1200, 60]; [0,600,30]; [0, 0, 0]; [0 0 0]];
% waypoints = [[0, 0, 0]; [600, 0, 30]; [600, 300, 30];[0,300,30]; [0, 0, 30]; [600,0,0]; [0,0,0]];
waypoints = [[0, 0, 0]; [600, 0, 100]; [600, 600, 100];[0,600,100]; [0, 0, 100]; [600,0,0]; [0,0,0]];
global control_effort;
control_effort = [];
global q1_z_arr;
q1_z_arr = [];
global dz_arr;
dz_arr = [];
global si_z_arr;
si_z_arr = [];
z_profile =[];
time = [];
counter = 0;
w_spd_ratio = 0.4;

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);

lw = 1;
tspan = 0:0.05:0.2;

% Straight Line Initial Condition
curr_x = 0;curr_x1 = 0;
curr_y = 0;curr_y1 = 0;
curr_si =0;curr_si1 =0;
curr_d = 0;curr_d1 = 0;
curr_z = 0;curr_z1 = 0;
curr_si_z = 0;curr_si_z1 = 0;
curr_d_z = 0;curr_d_z1 = 0; 
curr_integral_xy = 0; curr_integral_xy1 = 0;
curr_integral_z = 0; curr_integral_z1 = 0;
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
delta = 25;

wp = 1;
fprintf("Plotting trajectories\n");

%% Path following begins
last_wp = 6;
wp = 2;
while ( dist_wp > delta && wp <= last_wp)
    y0 = [curr_x curr_y curr_si curr_d curr_z curr_si_z curr_d_z curr_integral_xy curr_integral_z] ;
    y0_1 = [curr_x1 curr_y1 curr_si1 curr_d1 curr_z1 curr_si_z1 curr_d_z1 curr_integral_xy1 curr_integral_z1];
    [t,y] = ode45(@(t,y) odeFuncSLine3d_lqrIntegral(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,0), tspan, y0);
    [t1,y1] = ode45(@(t1,y1) odeFuncSLine3d_integral(t1,y1,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,w_spd_ratio), tspan, y0_1);
    curr_x = y(end,1); curr_x1 = y1(end,1);
    curr_y = y(end,2); curr_y1 = y1(end,2);
    curr_si = y(end,3); curr_si1 = y1(end,3);
    curr_d = y(end,4); curr_d1 = y1(end,4);
    curr_z = y(end,5); curr_z1 = y1(end,5);
    curr_si_z = y(end,6); curr_si_z1 = y1(end,6);
    curr_d_z = y(end,7); curr_d_z1 = y1(end,7);    
    curr_integral_xy = y(end,8); curr_integral_xy1 = y(end,9);
    curr_integral_z = y(end,8); curr_integral_z1 = y(end,9);
%     fprintf('x : %f,  y : %f,  z: %f, dist_wp: %f\n', curr_x,curr_y,curr_z,dist_wp);
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
    dist_wp1 = sqrt((w2_x - curr_x1)^2 + (w2_y - curr_y1)^2 + (w2_z - curr_z1)^2);
    
    if(dist_wp < delta)
        fprintf("waypoint reached: %d \n", wp);
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);
        
        pt = [curr_x,curr_y, 0]; pt1 = [curr_x1,curr_y1, 0];
        v1 = [w1_x w1_y 0];
        v2 = [w2_x w2_y 0];
        d = point_to_line(pt,v1,v2); d1 = point_to_line(pt1,v1,v2);

        pt = [curr_x,curr_y, curr_z]; pt1 = [curr_x1,curr_y1, curr_z1];
        v1 = [w1_x w1_y w1_z];
        v2 = [w2_x w2_y w2_z];
        d_3d = point_to_line(pt,v1,v2); d_3d1 = point_to_line(pt1,v1,v2);

        dz = sqrt((d_3d)^2 - d^2); dz1 = sqrt((d_3d1)^2 - d1^2);
%         w2_x = 1;
%         w2_y = 1;
%         w2_z = 600;
%         w1_x = 300; w1_y = 300; w1_z = 300;
        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
        dist_wp1 = sqrt((w2_x - curr_x1)^2 + (w2_y - curr_y1)^2 + (w2_z - curr_z1)^2);
        wp = wp + 1;
        curr_si_z = y(end,6); curr_si_z1 = y1(end,6);
        curr_d_z =  dz; curr_d_z1 =  dz1;          
        curr_si = y(end,3); curr_si1 = y1(end,3);
        curr_d = d; curr_d1 = d1;
    end
%     hold on
    plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
    z_profile = [z_profile,y(:,5)'];
    counter = counter + 1;
    plot3(y1(:,1),y1(:,2),y1(:,5),'-b','LineWidth',lw);
    if(wp <= last_wp+1)
        p1 = [waypoints(wp-1,1),waypoints(wp,1)];
        p2 = [waypoints(wp-1,2),waypoints(wp,2)];
        p3 = [waypoints(wp-1,3),waypoints(wp,3)];
        plot3(p1,p2,p3,'--k','LineWidth',1);
    end
    pause(0.01);
    hold on
    grid on
end
hold on
legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label
zlabel('Z(m)') % y-axis label

% figure
% plot(z_profile);
% title('z_profile');
% grid on
% % 
% figure
% plot(control_effort);
% title('control_effort');
% grid on
% %
% figure
% plot();
% title('control_effort');
% grid on

% 
% figure
% plot(q1_z_arr);
% title('Adaptive Gain');
% grid on
% 
% figure
% plot(dz_arr);
% title('Error');
% grid on
% 
% figure
% plot(si_z_arr);
% title('heading angle');
% grid on
