%Code for the ECE 620 Project #2

close all
clear all

A = [0 1 0 0; 0 -0.415 -0.011 0; 9.8 -1.43 -0.02 0; 0 0 1 0];
B = [0;6.27;9.8;0];
C = [0;-0.011;-0.02;0];
N=5;
R = 1;
Q = [1 0 0 0;0 0 0 0;0 0 0 0;0 0 0 1];

x0 = [10;0;0;20];
delta = 0;


t = 61;
dt =1;
z = 0;

[K,S,P] = lqr(A,B,Q,R);
K = K;
S = S;
P = P;

S(61) = S;
K(61) = K;
for k = t:1
    S(k) = transpose(A)*((((S(k-1))^-1)+(B*(R^-1)*(transpose(B))))^-1)*A+Q;
    K(k) = ((transpose(B)*S(k-1)*B + R)^-1)*(transpose(B))*S(k-1)*A;
end

x(1) = x0;
for k = 1:t
    u(k) = -K(k)*x(k);
    x(k+1) = A*x(k)+B*u(k)+C*z;
end