%% jMPC Toolbox Model Predictive Control Examples
% PIL Version

%   Jonathan Currie (C)
%   Control Engineering 2023

%% Dynamic System to Control via PIL
clc
clear
Gs = tf(2,[0.7 0.2 1]);
Gd = c2d(Gs,0.1);
% Create jSS Objects (Model Format)
Plant = jSS(Gd);
Model = Plant;  %No Model/Plant Mismatch

%Tuning
Np = 8;                 %Prediction Horizon
Nc = [4 2 2];           %Blocking Moves
uwt = 0.5;              %DeltaU Weights 
ywt = 0.5;              %Y Weights 
%Constraints
con = [];
con.u = [-inf inf 0.2]; %In1  [umin umax delumax]
con.y = [-inf 2];       %Out1 [ymin ymax]
%Estimator Gain
Kest = dlqe(Model);     %Discrete Observer with W,V = eye()

%Simulation Setup
T = 200;                %Length of Simulation
setp = ones(T,1);       %Setpoint
setp(75:150) = 0.5;

%Single Precision
opts = jMPCset('Single',1);

%Build MPC & Simulation
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%Assign Serial Device for PIL comms
simopts.opts.serialdevice = serial('COM6','BaudRate',1250000);

%% Generated Embedded MPC Controller
eopts = jMPCeset('arch','c2000','verifympc',1);
embed(MPC1,simopts,eopts);

%% PIL Verification Run
simpil = sim(MPC1,simopts,'pil')
plot(MPC1,simpil,'timing');

%% Compare against MATLAB
simMX = sim(MPC1,simopts,'mex');
compare(MPC1,simMX, simpil);