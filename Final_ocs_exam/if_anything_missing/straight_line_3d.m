clc; close all; clear all;

w1_x = 1; w1_y = 1; w1_z = 0;
w2_x = 300; w2_y = 300; w2_z = 300;

lw = 1;
tspan = 0:0.1:27;

% Straight Line Initial Condition
y0 = [50 0 0 0 0 0 0];
% y0 = [0 0 0 0];

% odeFuncSLine
[t,y] = ode45(@(t,y) odeFuncSLine3d(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z), tspan, y0);

figure
plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
grid on

w1_x = y(length(y(:,1)),1)
w1_y = y(length(y(:,1)),2)
w1_z = y(length(y(:,1)),5)
w2_x = 1; w2_y = 1; w2_z = 600;
y0 = [w1_x w1_y 0 0 w1_z 0 0];
[t,y] = ode45(@(t,y) odeFuncSLine3d(t,y,w1_x,w1_y,w1_z,w2_x,w2_y,w2_z), tspan, y0);
hold on
plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
grid on

% [t1,y1] = ode45(@(t1,y1) odeFuncSLineWind(t1,y1), tspan, y0);
% [t2,y2] = ode45(@(t2,y2) NLGL_Line(t2,y2,w1_x,w1_y,w2_x,w2_y), tspan, y0);


% Line Trajectory
% arr_x = (0:1:500);
% arr_y = (0:1:500);
% arr_z = (0:1:500);
% 
% plot3(arr_x(1,:),arr_y(1,:),arr_z(1,:),'--k'); 
% grid on

% hold on
% figure
% grid on
% hold on
% for i = 1:length(y(:,1))-1
% %     plot3(y(i:i+1,1),y(i:i+1,2),y(i:i+1,5),'-m','LineWidth',lw);
%     plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
% %     plot(y2(i:i+1,1),y2(i:i+1,2),'b','LineWidth',lw);
%     pause(0.01)
% end

% figure
% plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
% grid on

% legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
% xlabel('X(m)') % x-axis label
% ylabel('Y(m)') % y-axis label
% 
% d_arr = [];  
% for i = 1:length(y2(:,1))
%     pt = [y2(i,1),y2(i,2), 0];
%     v1 = [w1_x w1_y,0];
%     v2 = [w2_x w2_y,0];
%     d = point_to_line(pt,v1,v2);
%     d_arr = [d_arr,d];
% end
% 
% figure
% grid on
% plot(t2(:,1),d_arr(1,:),'b','LineWidth',lw);