clc;
clear all;
[x1,x2]=meshgrid(-4:0.2:4,-4:0.2:4);
dx1=x2;
dx2=-x2-x1;
quiver(x1,x2,dx1,dx2);
for i=-4:1:4
     for j=-4:1:4
         hold on
         fn=@(t,x)[x(2);-x(2)-1];
         [t,y]=ode45(fn,[0 10],[i j]);
         plot(y(:,1),y(:,2),'r');
     end
end
hold on
for i=-4:1:4
     for j=-4:1:4
         hold on
         fn=@(t,x)[x(2);-x(2)+1];
         [t,y]=ode45(fn,[0 10],[i j]);
         plot(y(:,1),y(:,2),'r');
     end
end
hold on
for i=-4:1:4
     for j=-4:1:4
         hold on
         fn=@(t,x)[x(2);-x(2)];
         [t,y]=ode45(fn,[0 10],[i j]);
         plot(y(:,1),y(:,2),'r');
     end
end
     
