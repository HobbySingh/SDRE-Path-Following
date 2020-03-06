clc;
clear all;
[x1,x2]=meshgrid(-8:0.2:8,-8:0.2:8);
dx1=x2;
if x1>1
    dx2=-x2-x1-1;
elseif x1<-1
    dx2=-x2-x1+1;
else
    dx2=-x2-x1;
end
quiver(x1,x2,dx1,dx2)
hold on
fn1=@(t,x)[x(2);-x(2)-x(1)];
[t,y]=ode45(fn1,[0 10],[1 0]);
plot(y(:,1),y(:,2),'r');
hold on
fn2=@(t,x)[x(2);-x(2)-x(1)+1];
[t,y]=ode45(fn2,[0 10],[3 3]);
plot(y(:,1),y(:,2),'g');
hold on
fn3=@(t,x)[x(2);-x(2)-x(1)-1];
[t,y]=ode45(fn3,[0 10],[-3 -3]);
plot(y(:,1),y(:,2),'y');
hold on
fn1=@(t,x)[x(2);-x(2)-x(1)];
[t,y]=ode45(fn1,[0 10],[-1 0]);
plot(y(:,1),y(:,2),'r');
