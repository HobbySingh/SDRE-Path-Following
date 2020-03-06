clc; close all; clear all;

w1_x = 0; w1_y = 0; w1_z = 0;
w2_x = 300; w2_y = 300; w2_z = 300;

lw = 1;
tspan = 0:0.1:57;


%Circle Initial Condition
y0 = [100 0 1.57 0 0 0 0];

%odeFuncCircleWind
[t,y] = ode45(@(t,y) odeFuncCircle3d(t,y), tspan, y0);
% [t1,y1] = ode45(@(t1,y1) odeFuncCircleWind(t1,y1), tspan, y0);


% Circle Trajectory
theta = linspace(0,2*pi);
tmp =[];
for i = 1:length(theta(1,:))-1
    tmp = [tmp; 100*cos(theta(i)),100*sin(theta(i)),2*theta(i)];
end
plot3(tmp(:,1),tmp(:,2),tmp(:,3),'--k');    
grid on
hold on
% hold on
% figure
% grid on
% hold on
% for i = 1:length(y(:,1))-1
%     plot3(y(i:i+1,1),y(i:i+1,2),y(i:i+1,5),'-m','LineWidth',lw);
% %     plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
% %     plot(y2(i:i+1,1),y2(i:i+1,2),'b','LineWidth',lw);
%     pause(0.01)
% end

% figure
plot3(y(:,1),y(:,2),y(:,5),'-m','LineWidth',lw);
grid on

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