%% MPCTools 3-DOF Helicopter
% Ref: J. �kesson (2006) MPCtools 1.0 Reference Manual, Lund Institute of
% Technology, page 24. Also similar to Quanser 3-DOF Helicopter except
% reordered states.

%   Copyright (C) 2011 Jonathan Currie (Control Engineering)

% You must run through each cell block in order

%% Linear Example
clc
clear
%Parameters
Je = 0.91;   % Moment of inertia about elevation axis 
la = 0.66;   % Arm length from elevation axis to helicopter body
Kf = 0.5;    % Motor Force Constant
Fg = 0.5;    % Differential force due to gravity and counter weight
Tg = la*Fg;  % Differential torque
Jp = 0.0364; % Moment of inertia about pitch axis
lh = 0.177;  % Distance from pitch axis to either motor
Jt = Je;     % Moment of inertia about travel axis

%Plant
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 -Fg*la/Jt 0;     
     0 0 0 0 0 1;
     0 0 0 0 0 0];
B = [0 0;
     Kf*la/Je Kf*la/Je;
     0 0;
     0 0;
     0 0;
     Kf*lh/Jp -Kf*lh/Jp];
 C = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
D = 0;

%Create jSS Object
Plant = jSS(A,B,C,D);

%Discretize Plant
Ts = 0.12;
Plant = c2d(Plant,Ts);
%Model
Model = Plant; %no model/plant mismatch

%Horizons
Np = 30; %Prediction Horizon
Nc = 10;  %Control Horizon
%Constraints
con.u = [-3 3 Inf;   
         -3 3 Inf];
con.y = [-0.6 0.6;       
         -pi  pi;       
         -1   1]; 
%Weighting
uwt = [1 1]'; 
ywt = [10 10 1]';
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Motor 1 Input','Motor 2 Input'},...
               'InputUnits',{'V','V'},...
               'OutputNames',{'Elevation Angle','Rotation Angle','Pitch Angle'},...
               'OutputUnits',{'rad','rad','rad'});

%Simulation Length
T = 300;
%Setpoint
setp = zeros(T,3);
setp(:,1) = 0.3;
setp(125:250,1) = -0.3;
setp(:,2) = 2;
setp(200:end,2) = 0;
%Initial values
Plant.x0 = [0 0 0 0 0 0]';
Model.x0 = [0 0 0 0 0 0]';

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);

%% Detailed Plot
plot(MPC1,simresult,'detail');

%% Nonlinear Example
clc
%Non Linear Plant
param = {Je,la,Kf,Fg,Tg,Jp,lh,Jt}; %parameter cell array
Plant = jNL(@nl_heli,C,param);
%Linearize Plant
u0 = [Tg/(2*Kf*la) Tg/(2*Kf*la)];
Model = linearize(Plant,u0);

%Build jSS object
Model = jSS(Model);
%Discretize model
Model = c2d(Model,Ts);

%Rebuild MPC & Simulation
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);

%Re-Simulate & Plot
simresultNL = sim(MPC1,simopts)
plot(MPC1,simresultNL);

%% Side by side plot
setp = simresult.setp;
y = simresult.plotvec.yp;
ynl = simresultNL.plotvec.yp;
k = 1:length(setp);
[kk,sp] = stairs(k,setp);

clf;
subplot(121)
plot(k,y,kk,sp,'k--')
axis([1 T+1 -1.2 2.5])
title('Linear Outputs: y_p(k)');
ylabel('Amplitude');

subplot(122)
plot(k,ynl,kk,sp,'k--')
axis([1 T+1 -1.2 2.5])
title('Nonlinear Outputs: y_p(k)')

