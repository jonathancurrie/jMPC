%% NLMPC TESTING
% Developer use only...

%% DIW Inverted Pendulum
clear
clc

%Parameters
M = 0.455;
m = 0.21;
l = 0.5*0.61;
g = 9.81;

%Linear Model
% A = [0   1   0             0;
%      0   0   -g*m/M        0;
%      0   0   0             1;
%      0   0   g*(M+m)/(l*M) 0]; 
% B = [    0;
%         1/M;
%          0;
%      -1/(l*M)];
C = [1 0 0 0;
     0 0 1 0];
% D = 0;

%Non Linear Plant & Model
param = {M,m,l,g}; %parameter cell array
Plant = jNL(@nl_pend,C,param);
Model = Plant;

%Provide Initial Linearized Model for MPC Controller
ubias = 0;
LinModel = linearize(Plant,ubias);
%Build jSS object
LinModel = jSS(LinModel);
%Discretize model
Ts = 0.05;
Model.lin_model = c2d(LinModel,Ts);

%-- MPC Setup --%
%Horizons & Time
Np = 40;
Nc = 10;
T = 300;

setp = zeros(T+1,2);
setp(:,1) = 1;
setp(125:250,1) = -1;
% setp(376:end,1) = -1;

%Constraints
con.u = [-10 10 10];
con.y = [-2    2;       
         -pi/4 pi/4]; 
con = [];
%Weighting
uwt = 2;
ywt = [2 1]';

%Estimator Gain
Kest = dlqe(Model.lin_model); 

%-- Build MPC & Simulation --%
opts.solver = 2;
opts.look_ahead = 0;
opts.Ts = Ts;

MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
MPC2 = jMPC(Model.lin_model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'Matlab');
sim2 = sim(MPC2,simopts,'Matlab');
compare(MPC1,sim1,sim2);

%% Lund 3-DOF Helicopter
clear
clc

%Parameters
Je = 0.91;
la = 0.66;
Kf = 0.5;
Fg = 0.5;
Tg = la*Fg;
Jp = 0.0364;
lh = 0.177;
Jt = 0.91;

% % Linear Model
% A = [0 1 0 0 0 0;
%      0 0 0 0 0 0;
%      0 0 0 1 0 0;
%      0 0 0 0 -Fg*la/Jt 0;     
%      0 0 0 0 0 1;
%      0 0 0 0 0 0;
%      ];
% B = [0 0;
%      Kf*la/Je Kf*la/Je;
%      0 0;
%      0 0;
%      0 0;
%      Kf*lh/Jp -Kf*lh/Jp];
 C = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
% D = zeros(3,2);

%Non Linear Plant & Model
param = {Je,la,Kf,Fg,Tg,Jp,lh,Jt}; %parameter cell array
Plant = jNL(@nl_heli,C,param);
Model = Plant;
% Model.Ts = 0.12;

%Provide Initial Linearized Model for MPC Controller
ubias = [0;0];
LinModel = linearize(Plant,ubias,[0;0;0;0;0;0]);
%Build jSS object
LinModel = jSS(LinModel);
%Discretize model
Ts = 0.05;
Model.lin_model = c2d(LinModel,Ts);

%Build jSS object
% Model = jSS(Model);
% %Discretize model
% Ts = 0.12;
% Model = c2d(Model,Ts);

%-- MPC Setup --%
%Horizons & Time
Np = 30;
Nc = 10;
T = 50;
%Setpoint (elevation,rotation,pitch)
setp = zeros(T+1,3);
setp(:,1) = 0.3;
setp(125:250,1) = -0.3;
setp(376:end,1) = -0.3;
setp(:,2) = 2;
setp(250:end,2) = 0;

%Constraints
con.u = [-3 3 Inf;   
         -3 3 Inf];
con.y = [-0.6    0.6;       
         -pi    pi;       
         -1   1];      
%Weighting
uwt = [1 1]';
ywt = [10 10 1]';

%Estimator Gain
%Kest = dlqe(Model); 

%-- Build MPC & Simulation --%
opts.solver = 2;
opts.look_ahead = 0;

MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,[],opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
plot(MPC1,simresult,'summary');


%% Quanser Quadruple Tank
clear
clc

%Parameters
A1  = 15.5179; %tank cross sectional area
A2  = 15.5179;
A3  = 15.5179;
A4  = 15.5179;
a1  = 0.1781; %outlet cross sectional area
a2  = 0.1781;
a3  = 0.1781;
a4  = 0.1781;
kc  = 1; 
g   = 981; 
k1  = 4.35; % pump coefficients
k2  = 4.39; 
g1  = 0.36; % ratio of allocated pump capacity between lower and upper tank
g2  = 0.36;

C =  eye(4); %measure all states

%Non Linear Plant
param = {A1,A2,A3,A4,a1,a2,a3,a4,g,g1,g2,k1,k2}; %parameter cell array
Plant = jNL(@nl_quadtank,C,param);

%Linearize Plant
Model = linearize(Plant,[7 7]);
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 3;
Model = c2d(Model,Ts);

%-- MPC Setup --%
%Horizons & Time
Np = 10;
Nc = 3;
T = 100;
%Setpoint (BTank1, BTank2)
setp = 15*ones(T+1,2);
setp(:,1) = 18;
setp(30:60,1) = 10;
% setp(80:100,2) = 10;

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
Kest = dlqe(Model,diag([100 100 100 100]),eye(size(Model.C,1))); 

%-- Build MPC & Simulation --%
opts.solver = 2;
opts.look_ahead = 0;

MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
plot(MPC1,simresult,'summary');