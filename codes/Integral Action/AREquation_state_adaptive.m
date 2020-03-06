clc
clear all

syms si si_p
syms q1 q2
syms d_xy vd

A = [0,1;0,0];
B = [0;(cos(si - si_p))];
Q = [q1,0;0,q2];
R = [1];
X = [d_xy;vd];
[K] = lqr(A,B,Q,R);

u = -K*X