clc; close all; clear all;

lw = 1;
tspan = 0:0.1:100;%50,75,105

r = 100;
speedx = 5;
speedz = 2;
v = 25;

tr_dist = 50;
%Helix Initial Condition 
y0 = [r 0 1.57 0 0 tr_dist 0 0 tr_dist 0 0];
curr_u_xy = 0;
curr_u_z = 0;
curr_d_xy = 0;
curr_d_z = 0;
center_x = 0; center_y = 0;

c = 0; % wind speed ratio , pink line
options = odeset('RelTol',1e-9,'AbsTol',1e-9,'OutputFcn',@(t,y,flag) circle_op_fx(t,y,flag,r,center_x,center_y,speedx,speedz,c,curr_u_xy,curr_u_z,curr_d_xy,curr_d_z),'Stats','on');
[t,y] = ode45(@(t,y) odeFuncMovingCircle3d2(t,y,r,center_x,center_y,speedx,speedz,c), tspan, y0, options);
d_xy1 = d_xy;
d_z1 = d_z;
u_xy1 = (u_xy);
u_z1 = (u_z);
sum(abs(d_xy1))

c = 0.2; % wind speed ratio , blue line
options1 = odeset('RelTol',1e-9,'AbsTol',1e-9,'OutputFcn',@(t1,y1,flag) circle_op_fx(t1,y1,flag,r,center_x,center_y,speedx,speedz,c,curr_u_xy,curr_u_z,curr_d_xy,curr_d_z));
[t1,y1] = ode45(@(t1,y1) odeFuncMovingCircle3d2(t1,y1,r,center_x,center_y,speedx,speedz,c), tspan, y0, options1);
d_xy2 = d_xy;
d_z2 = d_z;
u_xy2 = (u_xy);
u_z2 = (u_z);

c = 0.4; % wind speed ratio , red line
options2 = odeset('RelTol',1e-10,'AbsTol',1e-9,'OutputFcn',@(t2,y2,flag) circle_op_fx(t2,y2,flag,r,center_x,center_y,speedx,speedz,c,curr_u_xy,curr_u_z,curr_d_xy,curr_d_z));
[t2,y2] = ode45(@(t2,y2) odeFuncMovingCircle3d2(t2,y2,r,center_x,center_y,speedx,speedz,c), tspan, y0, options2);
d_xy3 = d_xy;
d_z3 = d_z;
u_xy3 = (u_xy);
u_z3 = (u_z);

theta = linspace(0,2*pi);

tmp2 = [];
ideal_x = [];
ideal_y = [];
ideal_z = [];
%fileID = fopen('real_pos.txt','w');

for i = 1:length(y(:,1))-1
    clf
%     grid on
    center_x = y(i,5);
    tmp =[];
    for j = 1:length(theta(1,:))-1
        tmp = [tmp; center_x + r*cos(theta(j)), center_y + r*sin(theta(j)), y(i,9)];
    end
    ideal_x = [ideal_x,center_x + r*cos((v*t(i,1))/r)];
    ideal_y = [ideal_y,center_y + r*sin((v*t(i,1))/r)];
    ideal_z = [ideal_z,y(i,6)];
    tmp2 = [tmp2; center_x, center_y, y(i,9)];
    
    curr_x = y(i,1); curr_y = y(i,2); curr_z = y(i,6); 
    %fprintf('x : %f,  y : %f,  z: %f \n', curr_x,curr_y,curr_z);

    plot3(tmp(:,1),tmp(:,2),tmp(:,3),'-.k'); 
    hold on
    plot3(y(1:i,1),y(1:i,2),y(1:i,6),'-m','LineWidth',lw); % without wind
    plot3(y1(1:i,1),y1(1:i,2),y1(1:i,6),'-b','LineWidth',lw); % with wind
    plot3(y2(1:i,1),y2(1:i,2),y2(1:i,6),'-r','LineWidth',lw); % with wind

    % ---------------- plotting end point --------------
    plot3(y(end,1),y(end,2),y(end,6),'om','LineWidth',lw+1); % without wind
    plot3(y1(end,1),y1(end,2),y1(end,6),'ob','LineWidth',lw+1); % with wind
    plot3(y2(end,1),y2(end,2),y2(end,6),'or','LineWidth',lw+1); % with wind
    
    plot3(tmp2(:,1),tmp2(:,2),tmp2(:,3) - tr_dist,'-g','LineWidth',lw+1); % for circle to follow
    plot3(tmp2(end,1),tmp2(end,2),tmp2(end,3) - tr_dist,'*k'); % for center of target
    
    plot3(tmp2(:,1),tmp2(:,2),tmp2(:,3),'-k'); % for circle to follow
    plot3(tmp2(end,1),tmp2(end,2),tmp2(end,3),'+b'); % for center of target
    
    plot3(ideal_x(end),ideal_y(end),ideal_z(end),'+k');
    grid on
    %fprintf(fileID,'%5d %5d\n',[y(end,1),y(end,2)]);

%     zlim([0 200])
    pause(0.001)
end

legend ('Path','v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Target Path','Location','northwest');
legend ('Path','v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('X(m)') % x-axis label
ylabel('Y(m)') % y-axis label
zlabel('Z(m)')

%% XY Error PLot
figure
subplot(2,1,1)

plot(t(:,1),d_xy1(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),d_xy2(:,1),'--b','LineWidth',lw);
plot(t2(:,1),d_xy3(:,1),'-r','LineWidth',lw);
% ylim([-1 1]);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('CrossTrack Error : d_{xy} (m)')
% title('(a)')

%% Z Error PLot
% figure
subplot(2,1,2)

plot(t(:,1),d_z1(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),d_z2(:,1),'--b','LineWidth',lw);
plot(t2(:,1),d_z3(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Altitude Error : d_{z} (m)')
% title('(b)')

%% Control Effort XY PLot

u_xy1 = abs(u_xy1);
u_z1 = abs(u_z1);
u_xy2 = abs(u_xy2);
u_z2 = abs(u_z2);
u_xy3 = abs(u_xy3);
u_z3 = abs(u_z3);

figure
subplot(2,1,1)

plot(t(:,1),u_xy1(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),u_xy2(:,1),'--b','LineWidth',lw);
plot(t2(:,1),u_xy3(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Control Effort : |u_{xy}| (m/sec^{2})')
% title('(a)')

%% Control Effort Z PLot
% figure
subplot(2,1,2)

plot(t(:,1),u_z1(:,1),'-.m','LineWidth',lw);
hold on
plot(t1(:,1),u_z2(:,1),'--b','LineWidth',lw);
plot(t2(:,1),u_z3(:,1),'-r','LineWidth',lw);
grid on

legend ('v_{w} = 0v','v_{w} = 0.2v','v_{w} = 0.4v','Location','northwest');
xlabel('Time (sec)');
ylabel('Control Effort : |u_{z}| (m/sec^{2})')
% title('(b)')
