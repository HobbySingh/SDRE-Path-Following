clc
clear all

syms p11 p12 p22 
syms q11 q22
syms si si_p
syms vd d_xy

A = [0,1;0,0];
P = [p11,p12;p12,p22];
B = [0;cos(si - si_p)];
R = [1];
Q = [q11,0;0,q22];
S = [0;0];
% care(A,B,Q,R,S,P)

first = A.'*P;
second = P*A;
third = -P*B*inv(R)*B.'*P;
fourth = Q;

are = first+second+third+fourth

eq1 = are(1,1)
eq2 = are(1,2)
eq3 = are(2,2)

p12 = solve(eq1,p12)

[p11,p22] = solve([eq2,eq3],[p11,p22])

X = [d_xy;vd];

u = -inv(R)*B.'*P*X
