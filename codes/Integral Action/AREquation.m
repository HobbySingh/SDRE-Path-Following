clc
clear all

syms p11 p12 p13 p22 p23 p33  
syms q11 q22 q33
syms d ddot intd

A = [0,1,0;0,0,0;1,0,0];
P = [p11,p12,p13;p12,p22,p23;p13,p23,p33];
B = [0;1;0];
R = [1];
Q = [q11*q11,0,0;0,q22*q22,0;0,0,q33*q33];
X = [d; ddot; intd];
first = A.'*P;
second = P*A;
third = -P*B*inv(R)*B.'*P;
fourth = Q;

% [K,S,e] = lqr(A,B,Q,R)

are = first+second+third+fourth

eq1 = are(1,1);
eq2 = are(1,2);
eq3 = are(1,3);
eq4 = are(2,2);
eq5 = are(2,3); 
eq6 = are(3,3);

[p11,p12,p13,p22,p23,p33] = solve([eq1,eq2,eq3,eq4,eq5,eq6],[p11,p12,p13,p22,p23,p33])

P = [p11(1),p12(1),p13(1);p12(1),p22(1),p23(1);p13(1),p23(1),p33(1)];
u = -inv(R)*B.'*P*X
