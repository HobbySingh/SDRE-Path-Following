clc; close all; clear all;

lw = 1;
tspan = 0:0.1:120;%50,75,105

r = 100;
speedx = 5;
speedz = 2;
v = 25;

%Helix Initial Condition 
y0 = [90 0 1.57 0 0 0 0 0 0 0 0];
center_x = 0; center_y = 0;

c = 0; % wind speed ratio , pink line
[t,y] = ode45(@(t,y) odeFuncMovingCircle3d2(t,y,r,center_x,center_y,speedx,speedz,c), tspan, y0);

c = 0.2; % wind speed ratio , blue line
[t1,y1] = ode45(@(t,y) odeFuncMovingCircle3d2(t,y,r,center_x,center_y,speedx,speedz,c), tspan, y0);

c = 0.4; % wind speed ratio , blue line
[t2,y2] = ode45(@(t,y) odeFuncMovingCircle3d2(t,y,r,center_x,center_y,speedx,speedz,c), tspan, y0);

theta = linspace(0,2*pi);

tmp2 = [];
ideal_x = [];
ideal_y = [];
ideal_z = [];
%fileID = fopen('real_pos.txt','w');

% for i = 1:length(y(:,1))-1
%     clf
%     grid on
%     center_x = y(i,5);
%     tmp =[];
%     for j = 1:length(theta(1,:))-1
%         tmp = [tmp; center_x + r*cos(theta(j)), center_y + r*sin(theta(j)), y(i,9)];
%     end
%     ideal_x = [ideal_x,center_x + r*cos((v*t(i,1))/r)];
%     ideal_y = [ideal_y,center_y + r*sin((v*t(i,1))/r)];
%     ideal_z = [ideal_z,y(i,6)];
%     tmp2 = [tmp2; center_x, center_y, y(i,9)];
%     
%     curr_x = y(i,1); curr_y = y(i,2); curr_z = y(i,6); 
% %     fprintf('x : %f,  y : %f,  z: %f \n', curr_x,curr_y,curr_z);
% 
%     plot3(tmp(:,1),tmp(:,2),tmp(:,3),'-.k'); 
%     hold on
%     plot3(y(1:i,1),y(1:i,2),y(1:i,6),'-m','LineWidth',lw); % without wind
%     plot3(y1(1:i,1),y1(1:i,2),y1(1:i,6),'-b','LineWidth',lw); % with wind
%     plot3(y2(1:i,1),y2(1:i,2),y2(1:i,6),'-r','LineWidth',lw); % with wind
% 
%     plot3(tmp2(:,1),tmp2(:,2),tmp2(:,3)/10,'-g','LineWidth',lw+1); % for circle to follow
%     plot3(tmp2(end,1),tmp2(end,2),tmp2(end,3)/10,'*k'); % for center of target
%     
% %     ---------------- plotting end point --------------
%     plot3(y(end,1),y(end,2),y(end,6),'om','LineWidth',lw+1); % without wind
%     plot3(y1(end,1),y1(end,2),y1(end,6),'ob','LineWidth',lw+1); % with wind
%     plot3(y2(end,1),y2(end,2),y2(end,6),'or','LineWidth',lw+1); % with wind
%     
%     plot3(tmp2(:,1),tmp2(:,2),tmp2(:,3),'-k'); % for circle to follow
%     plot3(tmp2(end,1),tmp2(end,2),tmp2(end,3),'+b'); % for center of target
%     
%     plot3(ideal_x(end),ideal_y(end),ideal_z(end),'+k');
%     grid on
% %     fprintf(fileID,'%5d %5d\n',[y(end,1),y(end,2)]);
% 
%     zlim([0 200])
%     pause(0.001)
% end
% xlim([-110,110])
% ylim([-150,150])
% zlim([0,150])
% 
% legend ('Path','v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Target Path','Location','northwest');
% legend ('Path','v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Target','Location','northwest');
% xlabel('X(m)') % x-axis label
% ylabel('Y(m)') % y-axis label
% zlabel('Z(m)')

% XY Error PLot
figure
subplot(2,1,1)
d_xy = ([0;y(2:end,10) - y(1:end-1,10)]);
d_xy1 = ([0;y1(2:end,10) - y1(1:end-1,10)]);
d_xy2 = ([0;y2(2:end,10) - y2(1:end-1,10)]);

plot(t(:,1),d_xy(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),d_xy1(:,1),'--b','LineWidth',lw);
plot(t2(:,1),d_xy2(:,1),'-r','LineWidth',lw);
% ylim([-1 1]);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('CrossTrack Error : d_{xy} (m)')
title('(a)')

% Z Error PLot
subplot(2,1,2)
d_z = ([0;y(2:end,11) - y(1:end-1,11)]);
d_z1 = ([0;y1(2:end,11) - y1(1:end-1,11)]);
d_z2 = ([0;y2(2:end,11) - y2(1:end-1,11)]);

plot(t(:,1),d_z(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),d_z1(:,1),'--b','LineWidth',lw);
plot(t2(:,1),d_z2(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Altitude Error : d_{z} (m)')
title('(b)')

%% Control Effort XY PLot
figure
subplot(2,1,1)
u = ([0;y(2:end,3) - y(1:end-1,3)]).^2*v^2;
u1 = ([0;y1(2:end,3) - y1(1:end-1,3)]).^2*v^2;
u2 = ([0;y2(2:end,3) - y2(1:end-1,3)]).^2*v^2;

plot(t(:,1),u(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),u1(:,1),'--b','LineWidth',lw);
plot(t2(:,1),u2(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Control Effort : u^{2}_{xy} (m/sec^{2})')
title('(a)')

%% Control Effort Z PLot
% figure
subplot(2,1,2)
u = ([0;y(2:end,7) - y(1:end-1,7)]).^2*v^2;
u1 = ([0;y1(2:end,7) - y1(1:end-1,7)]).^2*v^2;
u2 = ([0;y2(2:end,7) - y2(1:end-1,7)]).^2*v^2;

plot(t(:,1),u(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),u1(:,1),'--b','LineWidth',lw);
plot(t2(:,1),u2(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Control Effort : u^{2}_{z} (m/sec^{2})')
title('(b)')
