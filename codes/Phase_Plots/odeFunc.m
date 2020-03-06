function [dxdt] = odeFunc(t,x)

dxdt = zeros(2,1); 

vd = x(2);
k = 0.001;
q2 = 1;
vd_dot = -(exp(k*abs(x(1)))*x(1) + sqrt(2*exp(k*abs(x(1))) + q2)*x(2));

v = 25;
Rmin = 75;

if(abs(vd) > v)
    if (vd > 0)
        vd = v;
    else
        vd = -v;
    end
end
    

dxdt(1) = vd;  
dxdt(2) = vd_dot; 
