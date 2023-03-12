%% 3DOF Helicopter Setup for 3D MPC Simulation
% Ref: J. Åkesson (2006) MPCtools 1.0 Reference Manual, Lund Institute of
% Technology, page 24. Also similar to Quanser 3-DOF Helicopter except
% reordered states.

clear opts con

%Parameters
Je = 0.91;      % Moment of inertia about elevation axis [kgm^2]
la = 0.66;      % Arm length from elevation axis to helicopter body [m]
Kf = 0.5;       % Motor force constant
Fg = 0.5;       % Differential force due to gravity & counter weight [N]
Tg = la*Fg;     % Differential torque [Nm]
Jp = 0.0364;    % Moment of inertia about pitch axis [kgm^2]
lh = 0.177;     % Distance from pitch axis to either motor [m]
Jt = 0.91;      % Moment of inertia about rotation axis [kgm^2]
%Initial U
M1 = Tg/(2*Kf*la);  % Motor 1 voltage [V]
M2 = Tg/(2*Kf*la);  % Motor 2 voltage [V] 
%States & Outputs
% x1 = y1       - Elevation angle [rad]
% x2            - Elevation angular velocity [rad/s]
% x3 = y2       - Rotation angle [rad]
% x4            - Rotation angular velocity [rad/s]
% x5 = y3       - Pitch angle [rad]
% x6            - Pitch angular velocity [rad/s]
%Output Matrix
 C = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];

%Nonlinear Plant
param = {Je,la,Kf,Fg,Tg,Jp,lh,Jt};  %parameter cell array
Plant = jNL(@nl_heli,C,param);      %nl_heli.m contains model odes
%Linearize Plant
u0 = [M1;M2];
Model = linearize(Plant,u0,[],[],0);
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 0.2;
Model = c2d(Model,Ts);

%Horizons & Time
Np = 15;
Nc = 10;
T = 250;
%Setpoint (Elevation, Rotation)
setp = zeros(T,2);
setp(:,1) = 20*pi/180*ones(T,1);
setp(50:100,1) = -20*pi/180;
setp(101:150,1) = 5*pi/180;
setp(151:200,1) = -15*pi/180;
setp(100:160,2) = 100*pi/180;
setp(161:end,2) = -100*pi/180;
%Constraints
con.y = [-0.48  0.48;      %Limit elevation to +- 27.5 degrees
         -pi    pi;        %Limit rotation to +- 180 degrees
         -0.8  0.8];     %Limit pitch to +- 60 degrees
con.slack = [Inf Inf 5e3];           %Heavy penalty for soft constraints
%Weighting
uwt = [1 1]';
ywt = [5 5 0]';
%Estimator Gain
Kest = dlqe(Model); 

%Set Options
opts = jMPCset('InputNames',{'Motor 1 Input','Motor 2 Input'},...
               'InputUnits',{'V','V'},...
               'OutputNames',{'Elevation Angle','Rotation Angle','Pitch Angle'},...
               'OutputUnits',{'rad','rad','rad'});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

clear Je la Kf Fg Tg Jp lh Jt M1 M2 opts con param setp uwt ywt C Kest Nc Np u0