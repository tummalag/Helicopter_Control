% Homework #9

% Initialization 

clc;
clear all;
close all;
m = 1600;  % Mass in Kg
Fe = 1385;  % Max. Engine Force in N
Cd = 0.48;  % Aerodynamic drag coefficient in N-s/m
Vmax = 53.75; % Max. vehicle speed in m/s

% Reference speed to be tracked over [0,400]
for t=1:100
    r(t) = Vmax*t/(4*100);
end
r(101:150) = Vmax/4;
for t = 151:200
    r(t) = Vmax/4*(1 + (t-150)/50) ;
end
for t = 201:250
    r(t) = Vmax/2*(1 - (t-200)/100);
end
r(251:300) = Vmax/4;
for t = 301:375
    r(t) = Vmax/4*(1 - (t-300)/75);
end
r(376:400) = 0;

% Initializing q,r,k
q = 1;
s = zeros(1,400);
R = 1;
a = -Cd/m;
b = Fe/m;
t = 400;
dt = 1;
x = zeros(1,400);
v = zeros(1,400);
% Defining Control system Backward in Time
for i = t:-dt:2
    s(i-1) = 2*a*s(i) -  b^2*s(i)^2/R + q;
    v(i-1) = (a-b^2*s(i)/R)*v(i) + q*r(i);
end

for i = 1:dt:t-2
    k(i) = b*s(i)/R;
    u(i) = -k(i)*x(i) + b*v(i)/R;
    x(i+2) = -a*(x(i+1)^2) + b*u(i);
end
n = 1:1:400;
figure
plot(n,x,'-r',n,r,'-b')



