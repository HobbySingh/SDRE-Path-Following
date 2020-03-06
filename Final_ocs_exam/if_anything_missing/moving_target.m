clc; close all; clear all;

lw = 1;
tspan = 0:0.1:30;

% Straight Line Initial Condition
% y0 = [0 0 0.7854 0];
%Circle Initial Condition
% y0 = [100 0 1.57 0];

%odeFuncCircleWind
% [t,y] = ode45(@(t,y) odeFuncCircle(t,y), tspan, y0);
% [t1,y1] = ode45(@(t1,y1) odeFuncCircleWind(t1,y1), tspan, y0);
% odeFuncSLine
% [t,y] = ode45(@(t,y) odeFuncSLine(t,y), tspan, y0);
% [t1,y1] = ode45(@(t1,y1) odeFuncSLineWind(t1,y1), tspan, y0);
% [t2,y2] = ode45(@(t2,y2) NLGL_Line(t2,y2), tspan, y0);

%temporary
x0 = 100;si0 = 1.57;
yi =0;
% for x = 0:5:25
%     
    tspan = 0:0.1:150;
    y0 = [x0 yi si0 0 0];
%     origin_x = x;
%     origin_y = 0; 
    [t,y] = ode45(@(t,y) odeFuncCircle(t,yp ), tspan, y0);
    hold on
    grid on
    for i = 1:length(y(:,1))-1
        plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
        
        plot(y(i:i+1,5),0,'-m','LineWidth',lw);
        
    %     plot(y1(i:i+1,1),y1(i:i+1,2),'-b','LineWidth',lw);
    %     plot(y2(i:i+1,1),y2(i:i+1,2),'-.k','LineWidth',lw);
        pause(0.01)
    end
%     x0 = y(length(y(:,1)),1);
%     yi = y(length(y(:,1)),2);
%     si0 = y(length(y(:,1)),3);
% end

% grid on
% hold on
% Circle Trajectory
% theta = linspace(0,2*pi);
% tmp =[];
% for i = 1:length(theta(1,:))-1
%     tmp = [tmp; 100*cos(theta(i)),100*sin(theta(i))];
% end
% plot(tmp(:,1),tmp(:,2),'--k');    
 
% % Line Trajectory
% arr_x = (0:1:300);
% arr_y = (0:1:300);
% 
% plot(arr_x(1,:),arr_y(1,:),'--k'); 

% hold on
% for i = 1:length(y(:,1))-1
%     plot(y(i:i+1,1),y(i:i+1,2),'-m','LineWidth',lw);
% %     plot(y1(i:i+1,1),y1(i:i+1,2),'-b','LineWidth',lw);
% %     plot(y2(i:i+1,1),y2(i:i+1,2),'-.k','LineWidth',lw);
%     
%     pause(0.01)
% end
% 
% legend ('Path','AOGL without wind','AOGL with wind','Location','northwest');
% xlabel('X(m)') % x-axis label
% ylabel('Y(m)') % y-axis label