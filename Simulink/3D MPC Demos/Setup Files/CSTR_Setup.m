%% Seborg Continuously Stirred Reactor (CSTR)
% Ref: M. Henson and D. Seborg (1997), Nonlinear Process Control, Prentice
% Hall PTR, page 5.

clear opts con

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
Model = linearize(Plant,u0,xop,'ode15s',0);
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
%Disturbances (Always Delta)
mdist = zeros(T,2); %Measured disturbances
mdist(130:140,1) = 0.1; %Step disturbance of Caf
mdist(220:260,2) = -linspace(0,20,41); %Slow cooling of Tf
mdist(261:end,2) = -20; %Tf final
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

%Set Options
opts = jMPCset('InitialU',u0,... %Set Initial control input at linearization point
               'InputNames',{'Feed Concentration','Feed Temperature','Jacket Temperature'},...
               'InputUnits',{'mol/m^3','K','K'},...
               'OutputNames',{'Reactor Concentration of A','Reactor Temperature'},...
               'OutputUnits',{'mol/m^3','K'}); 

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp,[],ydist,mdist);

%3D Params
vrw = 'CSTR';
titles = {'Jacket Temperature [K]','Reactor Concentration [mol/m^3]','Reactor Temperature [K]'};
ylims = {[270 320],[0 1],[300 400]};
tlim = 10;

clear q V k0 E R H Cp Tc UA rho opts con param setp uwt ywt C Kest Nc Np u0