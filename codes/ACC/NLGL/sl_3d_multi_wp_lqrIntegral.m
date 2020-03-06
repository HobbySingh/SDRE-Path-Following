clc; clear all;
close all;

%% Initialization of params
waypoints = [[0, 0, 0]; [600, 0, 100]; [600, 600, 100];[0,600,100]; [0, 0, 100]; [600,0,0]; [0,0,0]];
% waypoints = [[0, 0, 100]; [600, 0, 150]; [600, 400, 150];[0,400,100]; [0, 800, 100]; [600,800,150]; [600,1200,150]; [0,1200,100]; [0,0,0]];
global control_effort;
control_effort = [];
global q1_z_arr;
q1_z_arr = [];
global dz_arr;
dz_arr = [];
global dxy_arr;
dxy_arr = [];

global si_z_arr;
global prev_d;
prev_d = 0;

si_z_arr = [];
z_profile =[];
time = [];
counter = 0;
w_spd_ratio = 0.2;

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);

lw = 1;
tspan = 0:0.05:0.2;

% Straight Line Initial Condition
curr_x = 0;
curr_y = 0;
curr_si =0;
curr_d = 0;
curr_z = 0;
curr_si_z = 0;
curr_d_z = 0;    
curr_integral_xy = 0;
curr_integral_z = 0;
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
delta = 25;

[sphere_x,sphere_y,sphere_z] = sphere;

%% Plotting Reference Trajectory
wp = 1;
fprintf("Plotting trajectories\n");

%% Path following begins

%% ------------------------LQR wind Plot---------------------------
last_wp = 6;
wp = 2;

arrow_plot = 0;
while ( dist_wp > delta && wp <= last_wp)
    arrow_plot = arrow_plot + 1;
    y0 = [curr_x curr_y curr_si curr_d curr_z curr_si_z curr_d_z curr_integral_xy curr_integral_z] ;
    [t,y] = ode45(@(t,y) odeFuncSLine3d_lqrIntegral(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,0.2), tspan, y0);
    curr_x = y(end,1);
    curr_y = y(end,2);
    curr_si = y(end,3);
    curr_d = y(end,4);
    curr_z = y(end,5);
    curr_si_z = y(end,6);
    curr_d_z = y(end,7);    

    curr_integral_xy = y(end,8);
    curr_integral_z = y(end,9);
    
%     fprintf('x : %f,  y : %f,  z: %f, dist_wp: %f\n', curr_x,curr_y,curr_z,dist_wp);
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
    
    if(dist_wp < delta)
        fprintf("waypoint reached: %d \n", wp);
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);
        
        pt = [curr_x,curr_y, 0];
        v1 = [w1_x w1_y 0];
        v2 = [w2_x w2_y 0];
        d = point_to_line(pt,v1,v2);

        pt = [curr_x,curr_y, curr_z];
        v1 = [w1_x w1_y w1_z];
        v2 = [w2_x w2_y w2_z];
        d_3d = point_to_line(pt,v1,v2);

        dz = sqrt((d_3d)^2 - d^2);
        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
        wp = wp + 1;
        curr_si_z = y(end,6);
        curr_d_z =  dz;
        curr_si = y(end,3);
        curr_d = d;
    end
%
    plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
    hold on
%     plot(y(:,1),y(:,2),'-m','LineWidth',lw);
    
    if(rem(arrow_plot,4) == 0)
        arrow_x = [y(end,1)];
        arrow_y = [y(end,2)];
        arrow_z = [y(end,5)];
        arrow_u = [25*cos(curr_si)];
        arrow_v = [25*sin(curr_si)];
        arrow_w = [25*sin(curr_si_z)];
%         quiver(arrow_x,arrow_y,arrow_u,arrow_v,0);
%         quiver3(arrow_x,arrow_y,arrow_z,arrow_u,arrow_v,arrow_w,0);
    end

    if(wp <= last_wp)
        p1 = [waypoints(wp-1,1),waypoints(wp,1)];
        p2 = [waypoints(wp-1,2),waypoints(wp,2)];
        p3 = [waypoints(wp-1,3),waypoints(wp,3)];
        plot3(p1,p2,p3,'--k');        
%         plot(p1,p2,'--k','LineWidth',1);
    end
    pause(0.01);
    grid on
end
hold on

%% ------------------------Wind Plot for NLGL---------------------------
curr_x = 0;
curr_y = 0;
curr_si =0;
curr_d = 0;
curr_z = 0;
curr_si_z = 0;
curr_d_z = 0;    
curr_integral_xy = 0;
curr_integral_z = 0;
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);

dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
last_wp = 6;
wp = 2;
while ( dist_wp > delta && wp <= last_wp)
    y0 = [curr_x curr_y curr_si curr_z curr_si_z] ;
    [t,y] = ode45(@(t,y) odeFuncSLine3d_NLGL(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z,0.2), tspan, y0);
    curr_x = y(end,1);
    curr_y = y(end,2);
    curr_si = y(end,3);
    curr_z = y(end,4);
    curr_si_z = y(end,5);    
    
%     fprintf('x : %f,  y : %f,  z: %f, dist_wp: %f\n', curr_x,curr_y,curr_z,dist_wp);
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
    
    if(dist_wp < delta)
        fprintf("waypoint reached: %d \n", wp);
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2); w1_z = wp_1(3);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2); w2_z = wp_2(3);
        
        pt = [curr_x,curr_y, 0];
        v1 = [w1_x w1_y 0];
        v2 = [w2_x w2_y 0];
        d = point_to_line(pt,v1,v2);

        pt = [curr_x,curr_y, curr_z];
        v1 = [w1_x w1_y w1_z];
        v2 = [w2_x w2_y w2_z];
        d_3d = point_to_line(pt,v1,v2);

        dz = sqrt((d_3d)^2 - d^2);

        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 + (w2_z - curr_z)^2);
        wp = wp + 1;
        curr_si_z = y(end,5);
        curr_si = y(end,3);
    end
%
    plot3(y(:,1),y(:,2),y(:,4),'-b','LineWidth',lw);
    hold on
%     plot(y(:,1),y(:,2),'-b','LineWidth',lw);

    if(wp <= last_wp + 1)
        p1 = [waypoints(wp-1,1),waypoints(wp,1)];
        p2 = [waypoints(wp-1,2),waypoints(wp,2)];
        p3 = [waypoints(wp-1,3),waypoints(wp,3)];
        plot3(p1,p2,p3,'--k');        
%         plot(p1,p2,'--k','LineWidth',1);
    end
    pause(0.01);
    grid on
end

