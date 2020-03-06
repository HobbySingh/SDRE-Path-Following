clc; close all; clear all;

lw = 1;
tspan = 0:0.1:47;

%Circle Initial Condition
y0 = [250 0 1.57 0 0];


center_x = 0; center_y = 0;
r = 250;
speedx = 0;

c = 0;
% odeFuncCirclewithoutWind
[t,y] = ode45(@(t,y) odeFuncMovingCircle(t,y,r,center_x,center_y,speedx,c), tspan, y0);

c = 0;
% odeFuncCirclewithWind
[t1,y1] = ode45(@(t,y) odeFuncMovingCircle2(t,y,r,center_x,center_y,speedx,c), tspan, y0);

grid on
% hold on
% Circle Trajectory
% theta = linspace(0,2*pi);
% tmp =[];
% for i = 1:length(theta(1,:))-1
%     tmp = [tmp; center_x + r*cos(theta(i)), center_y + r*sin(theta(i))];
% end
% plot(tmp(:,1),tmp(:,2),'--k');    

% hold on
theta = linspace(0,2*pi);

for i = 1:length(y(:,1))-1
    clf
    grid on
    center_x = y(i,5);
    tmp =[];
    for j = 1:length(theta(1,:))-1
        tmp = [tmp; center_x + r*cos(theta(j)), center_y + r*sin(theta(j))];
    end
    hold on
    plot(tmp(:,1),tmp(:,2),'-.k'); 
    plot(y(1:i,1),y(1:i,2),'-m','LineWidth',lw);
    plot(y1(1:i,1),y1(1:i,2),'-b','LineWidth',lw);
    
    pause(0.01)
end

legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label

%% multiple winds distance error plot circle
% d_arr = [];  
% for i = 1:length(y1(:,1))
%     dist = sqrt((y1(i,1))^2 + (y1(i,2))^2);
%     d = (dist - 100);
%     d_arr = [d_arr,d];
% end
% d_arr2 = [];  
% for i = 1:length(y2(:,1))
%     dist = sqrt((y2(i,1))^2 + (y2(i,2))^2);
%     d = (dist - 100);
%     d_arr2 = [d_arr2,d];
% end
% d_arr3 = [];  
% for i = 1:length(y3(:,1))
%     dist = sqrt((y3(i,1))^2 + (y3(i,2))^2);
%     d = (dist - 100);
%     d_arr3 = [d_arr3,d];
% end

% figure
% hold on
% plot(t1(:,1),d_arr(1,:),'b','LineWidth',lw);
% plot(t2(:,1),d_arr2(1,:),'m','LineWidth',lw);
% plot(t3(:,1),d_arr3(1,:),'k','LineWidth',lw);
% ylim([-5,12])
% grid on
% xlabel('Time(sec)') % x-axis label
% ylabel('Position Error: d(m)') % y-axis label
% legend ('vw = 0.25','vw = 0.35','vw = 0.45','Location','northwest');

