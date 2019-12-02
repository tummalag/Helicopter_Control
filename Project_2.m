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
    
Q = [ 0.00001  0  0  0 ;
      0  0  0  0 ;
      0 0 0 0;
      0 0 0 0.00001];
  
R =1000000;

z = 0; % hoizontal wind

del = zeros(61,1);

x = zeros(4,61);

x(:,1) = [ 10  ;  % pitch angle 'theta' in rads 
         0   ;  % pitch angle rate in rads/sec
         0   ;  % horizontal vel 'u' in m/s,
         20  ]; % horzontal dist 'x' in m
        
t = 61; % time 
dt = 1; % time period

% System Equation:
% Where, x is the state vector with the above components 
% del is the rotor thrust angle in rads
% z is the horizontal wind velocity in m/s

S_i = cell(1,61);
K = zeros(61,4);
S_i{61} = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
K(61,:) = [0,0,0,1];

%  To calculate reccati equation from backwards
for i = t:-dt:2
    S_i{i-1} = (A')*S_i{i} + S_i{i}*A - S_i{i}*B*inv(R)*B'*S_i{i} + Q;
    K(i-1,:) = inv(R)*B'*S_i{i};
end

% To calculate 'u' and bring to stable state
for i = 1:dt:t
    del(i,1) = -1*K(i,:)* x(:,i);
    x(:,i+1) = A*x(:,i) + B*del(i,1);   
    %x(:,i+1)   =  x(:,i)  + dt* x_d(:,i+1);
end

N =1:62;
figure
plot(N,x(1,N),'-b')





