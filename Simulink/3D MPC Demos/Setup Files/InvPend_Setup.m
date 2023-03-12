%% Inverted Pendulum Setup for 3D MPC Simulation
% Ref: David I Wilson, 3rd Assignment in Optimal Control, AUT
% University

clear opts con

%Parameters
M = 0.455;      % Mass of the cart [kg]
m = 0.21;       % Mass of the pendulum [kg]
l = 0.5*0.61;   % Distance to the center of mass of the pendulum [m]
g = 9.81;       % Gravitational constant [m/s^2]

%Initial U
f = 0;          % Force applied to the cart [N]
%States & Outputs
% x1 = y1       - Position of the cart [m]
% x2            - Velocity of the cart [m/s]
% x3 = y2       - Angle of the pendulum from vertical [rad]
% x4            - Angular velocity of the pendulum [rad/s]
%Output Matrix
C = [1 0 0 0;
     0 0 1 0];

%Nonlinear Plant
param = {M,m,l,g}; %parameter cell array
Plant = jNL(@nl_pend,C,param);
%Linearize Plant
u0 = f;
Model = linearize(Plant,u0,[],[],0);
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 0.05;
Model = c2d(Model,Ts);

%-- MPC Setup --%
%Horizons & Time
Np = 40;
Nc = 10;
T = 400;
%Setpoint (position,angle)
setp = zeros(T,1);
setp(:,1) = 1;
setp(125:250,1) = -1;
setp(376:end,1) = -1;

%Constraints
con.u = [-10 10 3];
con.y = [-5    5;       
         -pi/4 pi/4];    
con.slack = [Inf 5e3];
%Weighting
uwt = 2;
ywt = [1.5 0]'; %Don't control pendulum angle

%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Cart Force'},...
               'InputUnits',{'N'},...
               'OutputNames',{'Cart Position','Pendulum Angle'},...
               'OutputUnits',{'m','rad'});  

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

clear M m l g f opts con param setp uwt ywt C Kest Nc Np u0