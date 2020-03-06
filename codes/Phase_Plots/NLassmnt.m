clc;
clear all;
close all;
figure(1)
for i=-500:101:500
    for j=-25:6:25
        hold on
%         k = 0.001;
%         q1 = exp(k*abs(x(1)));
        q2 = 1;
%         u = -(exp(k*abs(x(1)))*x(1) + sqrt(2*exp(k*abs(x(1))) + q2)*x(2));
        
        %fn=@(t,x)[x(2);-(exp(k*abs(x(1)))*x(1) + sqrt(2*exp(k*abs(x(1))) + q2)*x(2))];
        [t,x]=ode45(@(t,x) odeFunc(t,x),[0 50],[i j]);
        a=1:1:50;
        plot(x(1,1),x(1,2),'*')
        plot(x(a,1),x(a,2),'k');
    end
end
grid on
% i=-160000;
% j=160000;
% k = 0.001;
% q2 = 1;
% fn=@(t,x)[x(2);-(exp(k*abs(x(1)))*x(1) + sqrt(2*exp(k*abs(x(1))) + q2)*x(2))];
% [t,y]=ode45(fn,[0 469],[i j]);
% a=1:1:469;
% plot(y(1,1),y(1,2),'*')
% plot(y(a,1),y(a,2),'k');
% grid on
% figure(2)
% for i=-0.05:.15:1
%     for j=-3:.15:1
%         hold on
%         fn=@(t,x)[x(1)+x(1)*x(2);-x(2)+x(2)^2+x(1)*x(2)-x(1)^3];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:30;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
% figure(3)
% for i=0:.1:1
%     for j=-1:.1:1
%         hold on
%         fn=@(t,x)[x(1)*(1-x(1)-2*((x(2)/(1+x(1)))));x(2)*(2-(x(2)/(1+x(1))))];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:10;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
% figure(4)
% for i=-1:.2:1
%     for j=-3:.2:1
%         hold on
%         fn=@(t,x)[x(2);-x(1)+x(2)*(1-x(1)^2+0.1*x(1)^4)];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:30;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
% figure(5)
% for i=-3:.2:3
%     for j=-3:.2:3
%         hold on
%         fn=@(t,x)[(x(1)-x(2))*(1-x(1)^2-x(2)^2);(x(1)+x(2))*(1-x(1)^2-x(2)^2)];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:30;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
% figure(6)
% for i=-3:.2:3
%     for j=-3:.2:3
%         hold on
%         fn=@(t,x)[-x(1)^3+x(2);x(1)-x(2)^3];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:30;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
% figure(7)
% for i=-3:.4:3
%     for j=-3:.4:3
%         hold on
%         fn=@(t,x)[-x(1)+x(2)+2*x(1)*x(2)+x(2)^2;x(1)-2*x(1)^2-x(1)*x(2)];
%         [t,y]=ode45(fn,[0 10],[i j]);
%         a=1:1:30;
%         plot(y(a,1),y(a,2),'k');
%     end
% end
