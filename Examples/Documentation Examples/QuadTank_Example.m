%% Quad Tank Controller
% Example jMPC Setup for a 4-Tank Level Controller

% Reference [1] "MPCtools 1.0 - Reference Manual by Johan Akesson, Lund
% Institute of Technology, 2006"
% Reference [2] "Coupled Water Tanks User Manual, QUANSER PDF, 2003"

%   Copyright (C) 2011 Jonathan Currie (Control Engineering)

% You must run through each cell block in order

%% Linear Example
clc
clear
% Parameters
%Tank cross sectional area
A1  = 15.5179; A2  = 15.5179; A3  = 15.5179; A4  = 15.5179;
%Outlet cross sectional area
a1  = 0.1781; a2  = 0.1781; a3  = 0.1781; a4  = 0.1781;
%Gravity
g   = 981; 
%Pump coefficients
k1  = 4.35; 
k2  = 4.39; 
%Ratio of allocated pump capacity between lower and upper tank
g1  = 0.36; 
g2  = 0.36;
%Steady state values
x0 = [15.0751 15.0036 6.2151 6.1003]; 
u0 = [7 7];
%Constants
T1 = A1/a1*sqrt((2*x0(1)/g));
T2 = A2/a2*sqrt((2*x0(2)/g));
T3 = A3/a3*sqrt((2*x0(3)/g));
T4 = A4/a4*sqrt((2*x0(4)/g));

%Plant
A = [-1/T1   0       A3/(A1*T3)   0;
     0       -1/T2   0            A4/(A2*T4);
     0       0       -1/T3        0;
     0       0       0            -1/T4];
    
B = [(g1*k1)/A1      0;
     0               (g2*k2)/A2;
     0               ((1-g2)*k2)/A3;
     ((1-g1)*k1)/A4  0];
    
C = eye(4);
D = 0;

%Create jSS Object
Plant = jSS(A,B,C,D);

%Discretize Plant
Ts = 3;
Plant = c2d(Plant,Ts);
%MPC Model
Model = Plant; %no model/plant mismatch

%Horizons & Time
Np = 10;
Nc = 3;
%Constraints
con.u = [0 12 6;   
         0 12 6];
con.y = [0 25;
         0 25;
         0 10;
         0 10];      
%Weighting
uwt = [8 8]';
ywt = [10 10 0 0]';

%Estimator Gain
Kest = dlqe(Model);

%Simulation Length
T = 250;
%Setpoints (Bottom Left, Bottom Right)
setp = 15*ones(T,2);
setp(100:end,1:2) = 20; 
setp(200:end,2) = 15;
%Initial values
Plant.x0 = [0 0 0 0]';
Model.x0 = [0 0 0 0]';

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest)
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);

%% Linear Example with Input Disturbance & Measurement Disturbance

%Disturbances
umdist = zeros(T,2);
umdist(80:end,1) = -1;
noise = randn(T,4)/30;
%Rebuild Simulation
simopts = jSIM(MPC1,Plant,T,setp,umdist,[],noise);

%Re Simulate & Plot
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);


%% Linear Example with Disturbances and no Kalman Filter

MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con)
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);
