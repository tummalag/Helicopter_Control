close all
clear all

A = [0 1 0 0; 0 -0.415 -0.011 0; 9.8 -1.43 -0.02 0; 0 0 1 0];
B = [0;6.27;9.8;0];
C = [0;-0.011;-0.02;0];

R = 1;
Q = [100 0 0 0;0 0 0 0;0 0 0 0;0 0 0 100];

x0 = [10;0;0;20];
delta = 0;


t = 61;
dt =1;
N=(1:dt:t);
z = 0;

[K,S,P] = lqr(A,B,Q,R);
S = S;
K = K;
S_k = cell(1,61);
K_k = cell(1,61);
S_k{1} = S;
K_k{1} = K;


for k = 2:t
    S_k{k} = transpose(A)*((((S_k{(k-1)})^-1)+(B*(R^-1)*(transpose(B))))^-1)*A+Q;
    K_k{k} = ((transpose(B)*S_k{(k-1)}*B + R)^-1)*(transpose(B))*S_k{(k-1)}*A;
end

x_k = cell(1,61);
x_k{1} = x0;
u = cell(1,61);
u_k{1} = K_k{t}*x_k{1};
x_k{2} =A*x_k{(1)}+B*u_k{1}+C*z;

for k = 2:t
    u_k{k} = -K_k{(62-k)}*(x_k{(k-1)});
    x_k{k+1} = A*x_k{(k)}+B*u_k{k}+C*z;
end