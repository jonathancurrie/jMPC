%% jMPC Toolbox Model Predictive Control Examples
% Nonlinear Plant Version

% A collection of example setups for learning the jMPC object with
% nonlinear plant simulations. Models are named according to author.

%   Jonathan Currie (C)
%   Control Engineering 2011

%% MPCTools 3-DOF Helicopter
% Ref: J. �kesson (2006) MPCtools 1.0 Reference Manual, Lund Institute of
% Technology, page 24. Also similar to Quanser 3-DOF Helicopter except
% reordered states.

% Identical setup to the linear example except using a nonlinear plant for
% the MPC simulation. This examples show how to use the jNL class to
% automatically build, find the steady state and linearize a collection of
% linear or nonlinear ODEs about an input operating point. Then with
% minimal changes, this can be substituted into the MPC simulation for a
% more accurate reflection of controller tuning.

% For a 3D Animation of this system see ThreeDOFHeli.mdl in
% jMPC\Simulink\3D MPC Demos
clear
clc
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
Model = linearize(Plant,u0);
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
         -1.05  1.05];     %Limit pitch to +- 60 degrees
con.slack = [Inf Inf 1e3];           %Heavy penalty for soft constraints
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
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'detail');

%% Quanser Quadruple Tank
% Ref: Quanser (2003) Coupled Tanks User Manual (www.quanser.com) +
% supplied MALTAB lab files for parameters.
% ODE Ref: J. �kesson (2006) MPCtools 1.0 Reference Manual, Lund
% Institute of Technology, page 20.

% This example follows closely to the linear example except all outputs can
% be measured from the ODE plant. Also shown is how to specify the ODE
% solver to be used when linearizing the plant, which will also propagate
% through to be used when simulating the ODE system.
clear
clc
%Parameters
A1  = 15.5179;  % Tank 1 cross sectional area [cm^2]
A2  = 15.5179;  % Tank 2 cross sectional area [cm^2]
A3  = 15.5179;  % Tank 3 cross sectional area [cm^2]
A4  = 15.5179;  % Tank 4 cross sectional area [cm^2]
a1  = 0.1781;   % Tank 1 outlet cross sectional area [cm^2]
a2  = 0.1781;   % Tank 2 outlet cross sectional area [cm^2]
a3  = 0.1781;   % Tank 3 outlet cross sectional area [cm^2]
a4  = 0.1781;   % Tank4 outlet cross sectional area [cm^2] 
g   = 981;      % Gravitational constant on Earth [cm/s^2]
k1  = 4.35;     % Pump 1 volumetric flow constant [cm^3/s/V]
k2  = 4.39;     % Pump 2 volumetric flow constant [cm^3/s/V]
g1  = 0.36;     % Ratio 1 of allocated pump capacity to lower tank
g2  = 0.36;     % Ratio 2 of allocated pump capacity to lower tank
%Initial U
P1 = 7;         % Pump 1 voltage [V]
P2 = 7;         % Pump 2 voltage [V] 
%States & Outputs
% x1 = y1       - Tank 1 Level (Bottom Left) [cm]
% x2 = y2       - Tank 2 Level (Bottom Right) [cm]
% x3 = y3       - Tank 3 Level (Top Left) [cm]
% x4 = y4       - Tank 4 Level (Top Right) [cm]
%Output Matrix
C =  eye(4);    % Measure all states

%Nonlinear Plant
param = {A1,A2,A3,A4,a1,a2,a3,a4,g,g1,g2,k1,k2}; %parameter cell array
Plant = jNL(@nl_quadtank,C,param);  %nl_quadtank.m contains model odes
%Linearize Plant
u0 = [P1;P2];
x0 = []; %unknown
Model = linearize(Plant,u0,x0,'ode23s');
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 3;
Model = c2d(Model,Ts);

%Horizons & Time
Np = 40;
Nc = 10;
T = 250;
%Setpoint (Bottom Left, Bottom Right)
setp = 15*ones(T,2);
setp(100:end,1:2) = 20; 
setp(200:end,2) = 15;
%Constraints
con.u = [0 12 Inf;   
         0 12 Inf];
con.y = [0 25;
         0 25;
         0 10;
         0 10];      
%Weighting
uwt = [8 8]';
ywt = [10 10 0 0]';
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Pump 1 Input','Pump 2 Input'},...
               'InputUnits',{'V','V'},...
               'OutputNames',{'Tank 1 (Bottom Left) Level','Tank 2 (Bottom Right) Level','Tank 3 (Top Left) Level','Tank 4 (Top Right) Level'},...
               'OutputUnits',{'cm','cm','cm','cm'});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');

%% DIW Inverted Pendulum on a Moving Cart
% Ref: David I Wilson, 3rd Assignment in Optimal Control, AUT
% University

% The following is a classic example of a nonlinear system controlled using
% this toolbox. The pendulum is linearized about an unstable operating
% point (vertical) and exhibits fast system dynamics. The tuning of the MPC
% controller is very sensitive and the user is encouraged to vary tuning
% weights and horizons to examine this.

% For a 3D Animation of this system see InvPend.mdl in
% jMPC\Simulink\3D MPC Demos
clear
clc
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
Model = linearize(Plant,u0);
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 0.05;
Model = c2d(Model,Ts);

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
con.u = [-10 10 2];
con.y = [-2    2;       
         -pi/4 pi/4];      
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
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');

%% Henson Continuously Stirred Reactor (CSTR)
% Ref: M. Henson and D. Seborg (1997), Nonlinear Process Control, Prentice
% Hall PTR, page 5.

% This more advanced example shows a typical chemical engieering process:
% the CSTR. The model ODEs contains one reaction pathway (A->B) with an
% energy balance. This example introduces measured disturbances which allow
% some of the plant inputs to not be controlled by MPC, but are still
% modelled as a disturbance as it can be measured. In this model, the feed
% concentration and temperature are both measured disturbances, while the
% reactor jacket temperature can be controlled by the MPC controller.

% Note all unmeasured disturbances within this toolbox are always given 
% as delta values, i.e. as a value +- of the current operating point.

% For a 3D Animation of this system see CSTR_Model.mdl in
% jMPC\Simulink\3D MPC Demos
clear
clc
%Parameters
q = 100;        % Volumetric flow rate [m^3/min]
V = 100;        % Volume in reactor [m^3]
k0 = 7.2e10;    % Pre-exponential nonthermal factor [1/min]
E = 7.2752e4;   % Activation energy in the Arrhenius Equation [J/mol]
R = 8.31451;    % Universal Gas Constant [J/mol-K]
H = 5e4;        % Heat of Reaction [J/mol]
Cp = .239;      % Heat capacity (J/g-K)
rho = 1000;     % Density (g/m^3)
UA = 5e4;       % Heat Transfer * Area [J/min-K]
%Initial U
CAf = 1;        % Feed Concentration [mol/m^3]
Tf = 350;       % Feed Temperature [K] 
Tc = 300;       % Coolant Temperature [K]
%States & Outputs
% x1 = y1       - Concentration in Reactor [mol/m^3]
% x2 = y2       - Reactor Temperature [K]
%Output Matrix
C = eye(2);

%Nonlinear Plant
param = {q,V,k0,E,R,H,Cp,rho,UA}; %parameter cell array
Plant = jNL(@nl_cstr,C,param);
%Linearize Plant
u0 = [CAf Tf Tc]';
xop = [0.5 350]'; %from book - unstable steady state
Model = linearize(Plant,u0,xop,'ode15s');
%Build jSS object & discretize
Model = jSS(Model);
Ts = 0.05;
Model = c2d(Model,Ts); 
%Set Measured Disturbances (Caf,Tf)
Model = SetMeasuredDist(Model,[1 2]); %Provide index of which inputs are mdist

%Horizons & Time
Np = 30;
Nc = [10 10 10];
T = 350;
%Setpoint (CA)
setp = zeros(T,1);
setp(:,1) = xop(2);
setp(50:end,1) = xop(2)+25;
setp(200:end,1) = xop(2)-25;
%Measured Disturbances
mdist = zeros(T,2); mdist(:,1) = CAf; mdist(:,2) = Tf;
mdist(130:140,1) = CAf+0.1; %Step disturbance of Caf
mdist(220:260,2) = Tf-linspace(0,20,41); %Slow cooling of Tf
mdist(261:end,2) = Tf-20; %Tf final
%Unmeasured Disturbances (Always Delta)
ydist = zeros(T,2); %Output Measurement Noise
% ydist(:,1) = 0.01*randn(T,1); %Concentration noise
% ydist(:,2) = 0.1*randn(T,1); %Temperature noise
%Constraints
con.u = [278.15 450 Inf];
con.y = [0   3;       
         278.15 450];   
%Weighting
uwt = 1;
ywt = [0 5]';
%Estimator Gain
Kest = dlqe(Model); 

%Set Initial states at linearization point
Plant.x0 = xop;
Model.x0 = xop;

%Set Options
opts = jMPCset('InitialU',u0,... %Set Initial control input at linearization point
               'InputNames',{'Feed Concentration','Feed Temperature','Jacket Temperature'},...
               'InputUnits',{'mol/m^3','K','K'},...
               'OutputNames',{'Reactor Concentration of A','Reactor Temperature'},...
               'OutputUnits',{'mol/m^3','K'},'QPWarmStart',1);    

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp,[],mdist,ydist);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');

%% Hahn Binary Distillation Column
% Ref: J. Hahn and T.F. Edgar (2002), An improved method for nonlinear 
% model reduction using balancing of empirical gramians, Computers and 
% Chemical Engineering, 26, pp. 1379-1397.

% Another classic Chemical Engineering problem is the binary distillation
% column as described by Hahn and Edgar. The feed flowrate and
% mole fraction are measured disturbances, while the reflux ratio can be
% manipulated by the controller in order to obtain the desired mole
% fractions in the output.
clear
clc
%Parameters
nTrays = 32;    % Number of Trays
feedTray = 17;  % Feed tray location
rVol = 1.6;     % Relative Volatility 
aTray = 0.25;   % Total Molar Holdup on each Tray
aCond = 0.5;    % Total Molar Holdup in the Condensor
aReb = 1.0;     % Total Molar Holdup in the Reboiler
%Initial U
fFeed = 24/60;  % Feed Flowrate [mol/min]
aFeed = 0.5;    % Feed Mole Fraction
RR = 3;         % Reflux Ratio
%States & Outputs
% x1 = y1          - Reflux Drum Liquid Mole Fraction of A
% x2               - Tray 1 Liquid Mole Fraction of A
% .
% x(feedTray) = y2 - Liquid Mole Fraction of A on Feed Tray
% .
% x(nTrays-1)      - Tray nTrays-1 Liquid Mole Fraction of A
% x(nTrays) = y3   - Reboiler Liquid Mole Fraction of A
%Output Matrix
C = zeros(3,nTrays);
C(1,1) = 1; C(2,feedTray) = 1; C(3,end) = 1;

%Nonlinear Plant
param = {feedTray,rVol,aTray,aCond,aReb,aFeed}; %parameter cell array
Plant = jNL(@nl_distil,C,param);
%Linearize Plant
u0 = [fFeed aFeed RR]';
x0 = []; %unknown
[Model,xop] = linearize(Plant,u0,x0,'ode15s');
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 1;
Model = c2d(Model,Ts);
%Set Measured Disturbances (fFeed,aFeed)
Model = SetMeasuredDist(Model,[1 2]);

%Horizons & Time
Np = 25;
Nc = 10;
T = 200;
%Setpoint (Reflux Drum)
setp = zeros(T,1);
setp(:,1) = xop(1);
setp(10:end,1) = xop(1)+0.02;
%Measured Disturbances
mdist = zeros(T,2); mdist(:,1) = fFeed; mdist(:,2) = aFeed;
mdist(60:end,2) = aFeed+0.01; %Small increase in feed mole fraction
mdist(140:end,1) = fFeed+1; %Small increase in feed flow rate
%Constraints
con.u = [0   10  1];
con.y = [0.92  1;
         0.48  0.52;
         0     0.1];
con.slack = [Inf 15 Inf];     
%Weighting
uwt = 1;
ywt = [10 0 0]';
%Estimator Gain
Kest = dlqe(Model); 

%Set Initial states at linearization point (don't start at 0)
Plant.x0 = xop;
Model.x0 = xop;

%Set Options
opts = jMPCset('InitialU',u0,... %Set Initial control input at linearization point
               'InputNames',{'Feed Flowrate','Feed Mole Fraction','Reflux Ratio'},...
               'InputUnits',{'mol/min','Fraction','Ratio'},...
               'OutputNames',{'Liquid Mole Fraction in Reflux Drum','Liquid Mole Fraction in Feed Tray','Liquid Mole Fraction in Reboiler'},...
               'OutputUnits',{'Fraction','Fraction','Fraction'}); 

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp,[],mdist,[]);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');