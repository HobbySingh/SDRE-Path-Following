clc; close all; clear all;

waypoints = [[0, 0]; [600, 0]; [600, 600];[0,600]; [0, 0];[0,0]];

wp_1 = waypoints(1,:);
w1_x = wp_1(1); w1_y = wp_1(2);

wp_2 = waypoints(2,:);
w2_x = wp_2(1); w2_y = wp_2(2);

lw = 1;
tspan = 0:0.05:0.2;

% Straight Line Initial Condition
y0 = [0 0 0 0];
curr_x = 0;
curr_y = 0;
curr_si =0;
curr_d = 0;
dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2 );
delta = 25;


%% ------------------------ MULTI WP CODE -----------------------

last_wp = 6;
wp = 2;
while ( dist_wp > delta && wp <= last_wp)
    
    y0 = [curr_x curr_y curr_si curr_d] ;
    [t,y] = ode45(@(t,y) odeFuncSLineWind_multi_wp(t,y,w1_x,w1_y,w2_x,w2_y,0.5), tspan, y0);
    curr_x = y(end,1);
    curr_y = y(end,2);
    curr_si = y(end,3);
    curr_d = y(end,4);

%     fprintf('x : %f,  y : %f,  z: %f, dist_wp: %f\n', curr_x,curr_y,curr_z,dist_wp);
    dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2);
    
    if(dist_wp <= delta)
        fprintf("waypoint reached: %d \n", wp);
        wp_1 = waypoints(wp,:);
        w1_x = wp_1(1); w1_y = wp_1(2);

        wp_2 = waypoints(wp+1,:);
        w2_x = wp_2(1); w2_y = wp_2(2);
        
        pt = [curr_x,curr_y 0];
        v1 = [w1_x w1_y 0];
        v2 = [w2_x w2_y 0];
        d = point_to_line(pt,v1,v2);

        dist_wp = sqrt((w2_x - curr_x)^2 + (w2_y - curr_y)^2);
        wp = wp + 1;
        
        curr_si = y(end,3);
        curr_d = d;
    end

    plot(y(:,1),y(:,2),'-m','LineWidth',lw);
    hold on
    if(wp <= last_wp + 1)
            
        p1 = [waypoints(wp-1,1),waypoints(wp,1)];
        p2 = [waypoints(wp-1,2),waypoints(wp,2)];
        
        plot(p1,p2,'--k','LineWidth',1);
    end
    pause(0.01);

    grid on
end
hold on
% legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label
zlabel('Z(m)') % y-axis label
