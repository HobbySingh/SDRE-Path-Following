clc; close all; clear all;

global u_arr;
u_arr = [];
w1_x = 0; w1_y = 0;
w2_x = 300; w2_y = 300;

lw = 1;
tspan = 0:0.1:17;

% Straight Line Initial Condition
y0 = [0 0 0.7854 0];

% odeFuncSLine
[t,y] = ode45(@(t,y) odeFuncSLine(t,y), tspan, y0);
[t1,y1] = ode45(@(t,y) odeFuncSLineWind(t,y,0.25), tspan, y0);
[t2,y2] = ode45(@(t,y) odeFuncSLineWind(t,y,0.35), tspan, y0);
[t3,y3] = ode45(@(t,y) odeFuncSLineWind(t,y,0.45), tspan, y0);

% Line Trajectory
arr_x = (0:1:300);
arr_y = (0:1:300);

plot(arr_x(1,:),arr_y(1,:),'--k'); 

grid on
hold on
for i = 1:length(y1(:,1))-1
    plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
    plot(y1(i:i+1,1),y1(i:i+1,2),'-b','LineWidth',lw);
    
    pause(0.01)
end

legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label

d_arr = [];  
for i = 1:length(y1(:,1))
    pt = [y1(i,1),y1(i,2), 0];
    v1 = [w1_x w1_y,0];
    v2 = [w2_x w2_y,0];
    d = point_to_line(pt,v1,v2);
    d_arr = [d_arr,d];
end

figure
grid on
plot(t1(:,1),d_arr(1,:),'b','LineWidth',lw);
xlabel('Time(sec)') % x-axis label
ylabel('Position Error(m)') % y-axis label

% figure
% grid on
% tmp = 1:1:length(u_arr);
% plot(tmp(1,:),u_arr(1,:),'b','LineWidth',lw);
% xlabel('Time(sec)') % x-axis label
% ylabel('Position Error(m)') % y-axis label
% % ylim([-10 1 ])