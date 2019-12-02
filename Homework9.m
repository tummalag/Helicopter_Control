% Homework #9

% Initialization 

clc;
clear all;
close all;
m = 1600;  % Mass in Kg
Fe = 1385;  % Max. Engine Force in N
Cd = 0.48;  % Aerodynamic drag coefficient in N-s/m
Vmax = 53.75; % Max. vehicle speed in m/s
dt = 1;


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


figure
plot(r)


% System Equation

%m*Sdd(t) = -*Cd*Sd(t)^2 + Fe*u(t)


