clc;
clear all;
[x1,x2]=meshgrid(-20:0.2:20,-20:0.2:20);
dx1=x2;
dx2=(-9.81/1)*sin(x1)-(0.5)*x2;
quiver(x1,x2,dx1,dx2)
hold on
fn=@(t,x)[x(2);-9.81*sin(x(1))-(0.5)*x(2)];
[t,y]=ode45(fn,[0 10],[0 4]);
plot(y(:,1),y(:,2),'r');
hold on
fn=@(t,x)[x(2);-9.81*sin(x(1))-(0.5)*x(2)];
[t,y]=ode45(fn,[0 10],[-3 3]);
plot(y(:,1),y(:,2),'g');
hold on
fn=@(t,x)[x(2);-9.81*sin(x(1))-(0.5)*x(2)];
[t,y]=ode45(fn,[0 10],[10 -10]);
plot(y(:,1),y(:,2),'y');