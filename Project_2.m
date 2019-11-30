% Helicopter Control 

% Initialization
clc;
clear all;
close all;

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
    
Q = [ 0  0  0  0 ;
      0  0  0  0 ;
      0 0 0 0;
      0 0 0 1];
  
R =1;
    
x(:,1) = [ 10  ;  % pitch angle 'theta' in rads 
         0   ;  % pitch angle rate in rads/sec
         0   ;  % horizontal vel 'u' in m/s,
         20  ]; % horzontal dist 'x' in m
     
t = 61; % time 
dt = 1; % time period

z = 0; % hoizontal wind


% System Equation:
% Where, x is the state vector with the above components 
% del is the rotor thrust angle in rads
% z is the horizontal wind velocity in m/s


for i = 1:dt:t
    x_d(:,t+1) = A*x(:,t) + B*del(:,t) + C*z;   
    x(:,t+1)   =  x(:,t)  + dt* x_d(:,t+1);
    %del(:,t+1) = -1*x_d(1,t+1);
end


[K,S,P] = lqr(x_d,Q,R);

K
S
P

figure
plot(t,x_d(1,:))





