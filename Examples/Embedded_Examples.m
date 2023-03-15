%% jMPC Toolbox Model Predictive Control Examples
% Embedded Version

%   Jonathan Currie (C)
%   Control Engineering 2023

%% Example 1: Oscillatory SISO Example
% A simple SISO model to get started with for embedded MPC
clc
%Model
Gs = tf(2,[0.7 0.2 1]);
Ts = 0.1;
%Convert to Discrete
Gd = c2d(Gs,Ts);
%Create a jSS Object
Plant = jSS(Gd);
%MPC Model
Model = Plant;

%Horizons & Time
Np = 8;                 %Prediction Horizon
Nc = [4 2 2];           %Control Horizon OR Blocking Moves
T = 200;                %Length of Simulation
%Setpoint
setp = ones(T,1);       %Setpoint
setp(75:150) = 0.5;
%Constraints
con = [];
con.u = [-inf   inf    0.2];  %in1  [umin umax delumax]
con.y = [-inf   2];           %out1 [ymin ymax]
%Weights
uwt = 0.5;              %U Weights (Larger = Penalize Delta U More)
ywt = 0.5;              %Y Weights (Larger = Penalize Setpoint-Y More)
%Estimator Gain
Kest = dlqe(Model);     %Discrete Observer with W,V = eye()

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);

%% Embed the controller
embed(MPC1,simopts)


%% Embed controller as single precision
clc
embeddedopts = jMPCeset('precision','single');
embed(MPC1,simopts,embeddedopts)

%% Generate MPC Testbench too (double vs double comparison)
clc
embeddedopts = jMPCeset('verifympc',1);
embed(MPC1,simopts,embeddedopts)

%% Embed controller as single precision (single vs single comparison)
clc
embeddedopts = jMPCeset('verifympc',1,'precision','single');
embed(MPC1,simopts,embeddedopts)

%% Embedded Single Precision Verified against Reference Double Precision
clc
embeddedopts = jMPCeset('tbprecision','double','precision','single','verifympc',1);
embed(MPC1,simopts,embeddedopts)

