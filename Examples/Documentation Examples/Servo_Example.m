%% Servomechanism Controller
% Example jMPC Setup for a Position Servomechanism Controller

% Reference "Model Predictive Control Toolbox 3, User's Guide", MATLAB

%   Copyright (C) 2011 Jonathan Currie (Control Engineering)

% You must run through each cell block in order

%% Linear Example (Un-Tuned)
clc
clear
%Parameters
k0 = 1280.2;    %Torisional Rigidity
kT = 10;        %Motor Constant
JM = 0.5;       %Motor Inertia
JL = 50*JM;     %Load Inertia
p = 20;         %Gear Ratio
BM = 0.1;       %Motor viscous friction
BL = 25;        %Load viscous friction
R = 20;         %Armature Resistance
%Inputs
% u1            - Motor Voltage [V]
%States & Outputs
% x1 = y1       - Load Angle [rad]
% x2            - Load Angular Velocity [rad/s]
% x3            - Motor Angle [rad]
% x4            - Motor Angular Velocity [rad/s]
%Unmeasured Outputs
% y2            - Load Torque [Nm]

%Plant
A = [0           1        0              0;
     -k0/JL      -BL/JL   k0/(p*JL)      0;
     0           0        0              1;
     k0/(p*JM)   0        -k0/(p^2*JM)   -(BM+kT^2/R)/JM];
B = [0;
     0;
     0;
     kT/(R*JM)];
C = [1 0 0 0];          %Plant Output
Cm = [1    0   0     0; %Model contains unmeasured outputs
     k0   0   -k0/p 0];
D = 0;

%Create jSS Plant
Plant = jSS(A,B,C,D);
Ts = 0.1;
Plant = c2d(Plant,Ts);
%Create jSS Model
Model = jSS(A,B,Cm,D);
Model = c2d(Model,Ts);
%Set Unmeasured Outputs in model
Model = SetUnmeasuredOut(Model,2);

%Horizons
Np = 10; %Prediction Horizon
Nc = 2;  %Control Horizon
%Constraints
con.u = [-220 220 1e2];   %in1 umin umax delumax   
con.y = [-inf   inf  ;    %out1 ymin ymax
         -78.5  78.5];    %out2 ymin ymax 
%Weighting
uwt = 0.1; 
ywt = [1 0]'; %do not control Torque
%Estimator Gain
Kest = dlqe(Model);

%Simulation Length
T = 300;
%Setpoint (Load Angle)
setp = ones(T,1);
setp(1:50) = 0;
%Initial values
Plant.x0 = [0 0 0 0]';
Model.x0 = [0 0 0 0]';

%Set Options
opts = jMPCset('InputNames',{'Motor Voltage'},...
               'InputUnits',{'V'},...
               'OutputNames',{'Load Angle','Load Torque'},...
               'OutputUnits',{'rad','Nm'});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);

%% Linear Example (Tuned)
clc
%Weighting
uwt = 0.05;
ywt = [2 0]';

%Rebuild MPC
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)

%Re-Simulate & Plot
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'detail'); 

%% Simulink Implementation
clc

%Re-Simulate & Plot
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');


