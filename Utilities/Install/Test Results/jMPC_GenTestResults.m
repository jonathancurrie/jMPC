function [dy,du] = jMPC_GenTestResults(varargin)
% Collection of standard models and setups used for post installation
% checks. Will automatically generate required .mat files in the default
% folder.

% Also functions as a Matlab->MEX and Matlab->Simulink test routine

if(~isempty(varargin)); simmode = varargin{1}; else simmode = 'save'; end

switch(lower(simmode))
    case 'save'        
    case 'mex'
    case 'simulink'
    otherwise
        error('Invalid mode supplied - options are ''save'' ''MEX'' and ''Simulink''');
end 

if(strcmpi(simmode,'save'))
    dec = input('\nThis will replace all saved MPC comparison results!\nAre you sure you want to continue? (y\\n): ','s');
    if(~strcmpi(dec,'y'));
        dy = []; du = [];
        return;
    end
end
    
clear varargin
saveloc = 'Utilities\Install\Test Results\mpc_test';
testno = 1;

%% Oscillatory SISO
%Model
Gs = tf(2,[0.7 0.2 1]);
Ts = 0.1;
%Convert to Discrete
Gd = c2d(Gs,Ts);
%Create a jSS Object
Model = jSS(Gd);

%Plant
Plant = Model;

%Horizons & Time
Np = 8;
Nc = [4 2 2];
T = 200;
% Setpoint
setp = ones(T,1);
setp(75:150) = 0.5;
% Disturbances
udist = zeros(T,1);
ydist = zeros(T,1);
mdist = zeros(T,0);

%Constraints
con.u = [-inf   inf    0.2];   %in1 umin umax delumax
con.y = [-inf   2];          %out1 ymin ymax

%Weighting
uwt = 0.5;
ywt = 0.5;
% Initial values
Plant.x0 = [0 0]';
Model.x0 = [0 0]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear Gs Ts Gd MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Rossiter MIMO Model
%Model
A = [0.6000   -0.4000    0.2000;
     1.0000    0.4000    0.4000;
     0.8000    1.0000    0.4000];
B = [0.4 0.3;
     0    -1;
     0   0.1]/2;
C =[1.0000   -2.2000    1.1200;
    0         1         1];
D = zeros(2,2);
Ts = 0.1;
%Create Object
Model = jSS(A,B,C,D,Ts);

%Plant
Plant = Model;

%Horizons & Time
Np = 10;
Nc = 5;
T = 10;
% Setpoint
setp = ones(T,2);
% Disturbances
udist = zeros(T,2);
ydist = zeros(T,2);
mdist = zeros(T,0);

%Constraints
con.u = [-2.4   2.7     0.7;   %in1 umin umax delumax
         -0.5   2.3     0.8];  %in2   
con.y = [-1.5   1.7;      %out1 ymin ymax
         -1.4   1.6];     %out2   
  
%Weighting
uwt = [1 1]';
ywt = [1 1]';
% Initial values
Plant.x0 = [0.3 0.2 -0.2]';
Model.x0 = [0.1 0 0.1]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear A B C D Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Maciejowski Cessna Model
%Linearized Continuous Time Cessna Model
A = [-1.2822   0  0.98   0;
         0     0   1     0;
     -5.4293   0 -1.8366 0;
     -128.2 128.2  0     0];
 
B = [-0.3; 0; -17; 0];

C = [   0      1  0 0;
        0      0  0 1;
     -128.2 128.2 0 0];

D = zeros(3,1);

%Build jSS object
Plant = jSS(A,B,C,D);
%Discretize model
Ts = 0.5;
Plant = c2d(Plant,Ts);

%-- MPC Setup --%
%MPC Model
Model = Plant; %no model plant mismatch
%Horizons & Time
Np = 10;
Nc = 5;
T = 40;
% Setpoint
setp = zeros(T,3);
setp(:,2) = 400*ones(T,1);
% Disturbances
udist = zeros(T,1);
ydist = zeros(T,3);
mdist = zeros(T,0);

%Constraints
con.u = [-0.2618  0.2618  0.5236];   %limit u to +- 15deg and rate to 30
con.y = [-0.35 0.35                  %limit pitch angle to +- 20deg
         -inf inf;
         -inf 30];
    
%Weighting
uwt = 1;
ywt = [1 1 1]';
% Initial values
Plant.x0 = [0 0 0 0]';
Model.x0 = [0 0 0 0]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear A B C D Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Quanser 3-DOF Helicopter
%Linearized Continuous Time Helicopter Model
A =     [0         0         0    1.0000         0         0;
         0         0         0         0    1.0000         0;
         0         0         0         0         0    1.0000;
         0         0         0         0         0         0;
         0         0         0         0         0         0;
         0   -1.2304         0         0         0         0];

B =     [0         0;
         0         0;
         0         0;
    0.0858    0.0858;
    0.5810   -0.5810;
         0         0];

C = [1     0     0     0     0     0;
     0     1     0     0     0     0;
     0     0     1     0     0     0];
 
D = [0     0;
     0     0;
     0     0];

%Build jSS object
Plant = jSS(A,B,C,D);
%Discretize model
Ts = 0.5;
Plant = c2d(Plant,Ts);

%-- MPC Setup --%
%MPC Model
Model = Plant; %no model plant mismatch
%Horizons & Time
Np = 15;
Nc = 10;
T = 250;
% Setpoint (elev,pitch,travel)
setp = zeros(T,3);
setp(:,1) = 20*pi/180*ones(T,1);
setp(50:100,1) = -20*pi/180;
setp(101:150,1) = 5*pi/180;
setp(151:200,1) = -15*pi/180;
setp(100:160,3) = 100*pi/180;
setp(161:end,3) = -100*pi/180;
% Disturbances
udist = zeros(T,2);
ydist = zeros(T,3);
mdist = zeros(T,0);

%Constraints
con.u = [-24 24 10;        %Limit of UPM is +-24V, rate unsure
         -24 24 10];
con.y = [-0.48 0.48;       %Limit elev to +- 27.5 degrees
         -1.05 1.05;       %Limit pitch to +- 60 degrees
         -pi    pi];       %Limit travel to +- 180 degrees
 
%Weighting
uwt = [0.5 0.5]';
ywt = [10 1 10]';
% Initial values
Plant.x0 = [0 0 0 0 0 0]';
Model.x0 = [0 0 0 0 0 0]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear A B C D Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% DIW Wood Berry
Gs = tf({12.8, -18.9; 6.6 -19.4}, ...
{ [16.7 1],[21 1]; [10.9 1],[14.4 1]});
Gs.ioDelayMatrix = [1,3; 7,3];

Ts = 1;
Gd = c2d(Gs,Ts);
Model = jSS(Gd);
Plant = Model;

Np = 10;
Nc = 5;
T = 30;
setp = [1 0.5];
% Disturbances
udist = zeros(T,2);
ydist = zeros(T,2);
mdist = zeros(T,0);

con.u = [-5 5 1;   
         -5 5 1];
con.y = [0 3;        
         0 2];      
uwt = [10 10]';
ywt = [1 1]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear Gs Gd Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;


%% MPC Tools Coupled Tanks
% %---Simulation Setup---%
%In this example we build the model assuming we can measure the top two
%tanks levels, however we will later remove these measurements for the
%actual plant model

%Linearized Discrete Time Four Tank Model
A =     [0.9366    0         0.0938    0; 
         0         0.9365    0         0.0946; 
         0         0         0.9030    0; 
         0         0         0         0.9022]; 

B =     [0.0977    0.0087;
         0.0087    0.0986;
         0         0.1721;
         0.1705    0];

C =  eye(4);
Cm = eye(2,4);
 
D = [0     0;
     0     0;
     0     0;
     0     0];

%Build jSS object
Ts = 1;
Plant = jSS(A,B,Cm,D,Ts);
Model = jSS(A,B,C,D,Ts);
%Set Unmeasured Outputs in model
Model = SetUnmeasuredOut(Model,[3 4]);

%Horizons & Time
Np = 40;
Nc = 10;
T = 250;
% Setpoint
setp = 15*ones(T,2);
setp(100:end,1:2) = 20; 
setp(200:end,2) = 15;
% Disturbances
udist = zeros(T,2);
ydist = zeros(T,2);
mdist = zeros(T,0);

%Constraints
con.u = [0 12 3;   
         0 12 3];   
con.y = [0 25;
         0 25
         0 10
         0 10];    

%Weighting
uwt = [8 8];
ywt = [10 10 0 0]; %don't control top two tanks, but keep constraints

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear A B C Cm D Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Rossiter SISO DISTURBANCES
%Model
Gs = tf({2,1}, {[0.7 0.2 1],[4 1]});
Ts = 0.1;
%Convert to Discrete
Gd = c2d(Gs,Ts);
%Create a jSS Object
Model = jSS(Gd);
%Set Measured Disturbances in model
Model = SetMeasuredDist(Model,2);

%Plant
Plant = Model;

%Horizons & Time
Np = 8;
Nc = [4 2 2];
T = 200;
% Setpoint
setp = ones(T,1);
setp(75:150) = 0.5;
%Disturbances
udist = zeros(T,2);
ydist = zeros(T,1);
mdist = zeros(T,1);
mdist(120:180,1) = 3;

%Constraints
con.u = [-inf   inf    0.2];   %in1 umin umax delumax
con.y = [-inf   2];          %out1 ymin ymax

%Weighting
uwt = 0.5;
ywt = 0.5;

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear Gs Gd Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Seborg Continuously Stirred Reactor (CSTR)
A = [0.9004  -0.0003408;
     0.8385        0.67];
B = [0.09493  -1.853e-005  -3.877e-005;
     -0.04561       0.0824       0.1724];
C = eye(2);
D = 0;
Ts = 0.1;

%Build jSS object
Model = jSS(A,B,C,D,Ts); Plant = Model;
%Set Measured Disturbances (Caf,Tf)
Model = SetMeasuredDist(Model,[1 2]);

%-- MPC Setup --%
%Horizons & Time
Np = 30;
Nc = [10 10 10];
T = 350;
%Setpoint (CA)
setp = zeros(T,1);
setp(50:end,1) = 0.01;

%Disturbances (Delta)
udist = zeros(T,3);
mdist = zeros(T,2); %Measured disturbances
mdist(130:140,1) = 0.01; %Step disturbance of Caf
mdist(220:260,2) = -linspace(0,10,41); %Slow cooling of Tf
mdist(261:end,2) = -10; %Tf final
ydist = zeros(T,2); %Output Measurement Noise
ydist(:,1) = 0.0001*randn(T,1); %Concentration noise
ydist(:,2) = 0.001*randn(T,1); %Temperature noise

%Constraints
con.u = [-30 30 1];
con.y = [-1   1;       
         -50 50];   

%Weighting
uwt = 0.1;
ywt = [5 0]';

%Estimator Gain
Kest = dlqe(Model); 

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear A B C D Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% DIW Inverted Pendulum on a moving Cart
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

%Non Linear Plant
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
%Disturbances
udist = zeros(T,1);
ydist = zeros(T,2);
mdist = zeros(T,0);

%Constraints
con.u = [-10 10 2];
con.y = [-2    2;       
         -pi/4 pi/4];      
%Weighting
uwt = 2;
ywt = [1.5 0]'; %Don't control pendulum angle

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts = jMPCset();
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))    
    clear M m l g f C u0 Ts param MPC1 simopts
    save([saveloc num2str(testno)]);     
else
    if(strcmpi(simmode,'Simulink'))
        sim2 = sim(MPC1,simopts,simmode);
        [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
    end
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Seborg Continuously Stirred Reactor (CSTR)
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
%Build jSS object
Model = jSS(Model);
%Discretize model
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
udist = zeros(T,3);
ydist = zeros(T,2); %Output Measurement Noise
% ydist(:,1) = 0.01*randn(T,1); %Concentration noise
% ydist(:,2) = 0.1*randn(T,1); %Temperature noise
%Constraints
con.u = [150 450 20];
con.y = [0   3;       
         250 450];   
%Weighting
uwt = 1;
ywt = [0 5]';
%Estimator Gain
Kest = dlqe(Model); 

%Set Initial states at defined point
Plant.x0 = xop;
Model.x0 = xop;
%Set Initial control input at linearization point
opts = jMPCset('InitialU',u0);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'save'))
    clear F V k0 E R H Cp rho UA CAf Tf Tj C param u0 x0 xop Ts MPC1 simopts
    save([saveloc num2str(testno)]);    
else
    if(strcmpi(simmode,'Simulink'))
        sim2 = sim(MPC1,simopts,simmode);
        [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
    end
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% DIW Binary Distillation Column
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
[Model,xop] = linearize(Plant,u0,x0,'ode15s',0);
%Build jSS object
Model = jSS(Model);
%Discretize model
Ts = 1;
Model = c2d(Model,Ts);
%Set Measured Disturbances (fFeed,aFeed)
Model = SetMeasuredDist(Model,[1 2]);

%Horizons & Time
Np = 25;
Nc = [3 3 3 6 10];
T = 200;
%Setpoint (Reflux Drum)
setp = zeros(T,1);
setp(:,1) = xop(1);
setp(10:end,1) = xop(1)+0.02;
%Measured Disturbances
mdist = zeros(T,2); mdist(:,1) = fFeed; mdist(:,2) = aFeed;
mdist(60:end,2) = aFeed+0.01; %Small increase in feed mole fraction
mdist(140:end,1) = fFeed+1; %Small increase in feed flow rate
%Unmeasured Disturbances
udist = zeros(T,3);
ydist = zeros(T,3);
%Constraints
con.u = [0   10  1];
con.y = [0.92  Inf;
         0.48  0.52;
         0     0.1];
con.slack = [10 10 10]; %soft output constraints with heavy penalty   
%Weighting
uwt = 1;
ywt = [10 0 0]';
%Estimator Gain
Kest = dlqe(Model); 

%Set Initial states at linearization point (don't start at 0)
Plant.x0 = xop;
Model.x0 = xop;
%Set Initial control input at linearization point
opts = jMPCset('InitialU',u0);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,udist,mdist,ydist);

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Matlab');
if(strcmpi(simmode,'Simulink'))
    sim2 = sim(MPC1,simopts,simmode);
    [dy(testno),du(testno)] = compare(MPC1,simresult,sim2);    
end
clear opts con udist ydist mdist Plant Model
testno = testno+1;

%% Plot Accuracy
if(strcmpi(simmode,'save'))
    dy = []; du = [];
else
    figure(1); clf;
    bar(abs(log10([du' dy'])))
    title([simmode ' Simulation Accuracy']);
    xlabel('Simulation (Blue Input, Red Output)');
    ylabel('Abs Log10(norm(error))');   
end
