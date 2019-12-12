% Helicopter Control 

% Initialization
clc;clear all;close all;

A = [0   ,    1    ,    0    ,  0  ;
     0   ,  -0.415 , -0.011  ,  0  ;
     9.8 ,  -1.43  ,  -0.02  ,  0  ;
     0   ,    0    ,    1    ,  0  ];
     
B = [  0   ;
      6.27 ;        
      9.8  ;
       0   ];
   
C = [   0    ;
      -0.011 ;
      -0.02  ;
        0    ];
z = 0;              % hoizontal wind
del = zeros(60,1); 
del_r = 9;  
x = zeros(4,60);
% Reference Values
x_r = [ 15 ;
         8 ;
         2 ; 
        25];
x(:,1) = [ 10   ;  % pitch angle 'theta' in rads 
            0   ;  % pitch angle rate in rads/sec
            0   ;  % horizontal vel 'u' in m/s,
           20  ];  % horzontal dist 'x' in m

Q =  [10  0  0  0 ;
       0  4  0  0 ;
       0  0  50 0 ;
       0  0  0  800];
  
R =1000000000;

t = 60;         % time 
dt = 0.001;      % sampling time period
N =1:dt:t;
n = length(N);

[K,S,P] = lqr(A,B,Q,R);

% System Equation:
% Where, x is the state vector with the above components 
% del is the rotor thrust angle in rads
% z is the horizontal wind velocity in m/s
% To calculate 'u' and bring to stable state
for i = 1:1:length(1:dt:t)
    del(i,1) = -K*x(:,i);
    del(i,1) =min(max(del(i,1),-del_r),del_r);
    y(:,i+1) = x(:,i) + dt*(A*x(:,i) + B*del(i,1) + C*z);   
    x(:,i+1) = min(max(y(:,i+1),-x_r),x_r);
end

% Plots
% The following plot represents the state variable
subplot(2,2,1);
plot(N,x(1,1:n),'-b')
title('Plot 1: Pitch angle \theta "x1" ')
subplot(2,2,2);
plot(N,x(2,1:n),'-b')
title('Plot 2: Pitch Angle rate "x2" ')
subplot(2,2,3);
plot(N,x(3,1:n),'-b')
title('Plot 3: Horizontal velocity "x3" ')
subplot(2,2,4);
plot(N,x(4,1:n),'-b')
title('Plot 4: Horizontal distance "x4" ')

% The following plot represents the control value
figure(2)
plot(N,del(1:n,1),'-b')
title('Plot 5: Rotor thrust angle "u" ')

% This plot represents the poles of the system
figure(3)
plot(real(P),imag(P),'b*')
title('Poles')
ax = gca;
ax.XAxisLocation = 'origin';
ax.YAxisLocation = 'origin';

