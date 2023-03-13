%% Distillation Column
% Ref J. Hahn & T. F. Edgar (2002)
clc
clear

%% Nonlinear Example
%Parameters
nTrays = 32;   % Number of Trays
feedTray = 17; % Feed tray location
rVol = 1.6;    % Relative Volatility 
aTray = 0.25;  % Total Molar Holdup on each Tray
aCond = 0.5;   % Total Molar Holdup in the Condensor
aReb = 1.0;    % Total Molar Holdup in the Reboiler
aFeed = 0.5;   % Feed Mole Fraction
%Output Matrix
C = zeros(3,nTrays);
C(1,1) = 1; C(2,feedTray) = 1; C(3,end) = 1;
%Nonlinear Plant
param = {feedTray,rVol,aTray,aCond,aReb,aFeed}; %parameter cell array
Plant = jNL(@nl_distil,C,param);

%Initial U
fFeed = 24/60; % Feed Flowrate [mol/min]
aFeed = 0.5;   % Feed Mole Fraction
RR = 3;        % Reflux Ratio
%Linearize Plant
u0 = [fFeed aFeed RR]';
x0 = []; %unknown operating point
[Model,xop] = linearize(Plant,u0,x0,'ode15s');

%Build jSS object & discretize
Model = jSS(Model);
Ts = 1;
Model = c2d(Model,Ts);

%Set Measured Disturbances (fFeed,aFeed)
Model = SetMeasuredDist(Model,[1 2]); %Provide index of which inputs are mdist

%Horizons
Np = 25; %Prediction Horizon
Nc = 10; %Control Horizon

%Constraints
con.u = [0  10  1];  
con.y = [0.92  1;
         0.48  0.52;
         0     0.1];
    
%Weighting
uwt = 1; 
ywt = [10 0 0]';  

%Estimator Gain
Kest = dlqe(Model);

%Simulation Length
T = 200;

%Setpoint (Reflux Drum)
setp = zeros(T,1);
setp(:,1) = xop(1);
setp(10:end,1) = xop(1)+0.02;

%Measured Disturbances
mdist = zeros(T,2); mdist(:,1) = fFeed; mdist(:,2) = aFeed;
mdist(60:end,2) = aFeed+0.01;   %Small increase in feed mole fraction
mdist(140:end,1) = fFeed+1;     %Small increase in feed flow rate

%Set Initial values at linearization point
Plant.x0 = xop;
Model.x0 = xop;

%Set Options
opts = jMPCset('InitialU',u0,...  %Set initial control input at linearization point
               'InputNames',{'Feed Flowrate','Feed Mole Fraction','Reflux Ratio'},...
               'InputUnits',{'mol/min','Fraction','Ratio'},...
               'OutputNames',{'Liquid Mole Fraction in Reflux Drum','Liquid Mole Fraction in Feed Tray',...
               'Liquid Mole Fraction in Reboiler'},...
               'OutputUnits',{'Fraction','Fraction','Fraction'});
           
%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp,[],mdist);   

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink');
plot(MPC1,simresult,'detail');