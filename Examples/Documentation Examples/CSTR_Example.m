%% CSTR Example
% Ref M. Henson & D.Seborg (1997)


%% Nonlinear Example
clc
clear
%Parameters
q = 100;     % Volumetric flow rate [m^3/min]
V = 100;     % Volume in reactor [m^3]
k0 = 7.2e10; % Pre-exponential nonthermal factor [1/min]
E = 7.2752e4;% Activation energy in the Arrhenius Equation [J/mol]
R = 8.31451; % Universal Gas Constant [J/mol-K]
H = 5e4;     % Heat of Reaction [J/mol]
Cp = .239;   % Heat capacity (J/g-K)
rho = 1000;  % Density (g/m^3)
UA = 5e4;    % Heat Transfer * Area [J/min-K]
%Output Matrix
C = eye(2);
%Nonlinear Plant
param = {q,V,k0,E,R,H,Cp,rho,UA};  %parameter cell array
Plant = jNL(@nl_cstr,C,param); 

%Initial U
CAf = 1; % Feed Concentration [mol/m^3]
Tf = 350; % Feed Temperature [K] 
Tc = 300; % Coolant Temperature [K]
%Linearize Plant
u0 = [CAf Tf Tc]';
xop = [0.5 350]'; %unstable operating point [Ca Tr]
Model = linearize(Plant,u0,xop,'ode15s');

%Build jSS object & discretize
Model = jSS(Model);
Ts = 0.05;
Model = c2d(Model,Ts);

%Set Measured Disturbances (Caf,Tf)
Model = SetMeasuredDist(Model,[1 2]); %Provide index of which inputs are mdist

%Horizons
Np = 30;         %Prediction Horizon
Nc = [10 10 10]; %Blocking Moves
%Constraints
con.u = [278.15 450 20];  
con.y = [0      3;
         278.15 450];
%Weighting
uwt = 1; 
ywt = [0 5]';  

%Estimator Gain
Kest = dlqe(Model);
%Simulation Length
T = 350;

%Setpoint (CA)
setp = zeros(T,1);
setp(:,1) = xop(2);
setp(50:end,1) = xop(2)+25;
setp(200:end,1) = xop(2)-25;
%Measured Disturbances (Caf Tf)
mdist = zeros(T,2); mdist(:,1) = CAf; mdist(:,2) = Tf;
mdist(130:140,1) = CAf+0.1;              %Step disturbance of Caf
mdist(220:260,2) = Tf-linspace(0,20,41); %Slow cooling of Tf
mdist(261:end,2) = Tf-20;                %Tf final

%Set Initial values at linearization point
Plant.x0 = xop;
Model.x0 = xop;
%Set Options
opts = jMPCset('InitialU',u0,...  %Set initial control input at linearization point
               'InputNames',{'Feed Concentration','Feed Temperature','Jacket Temperature'},...
               'InputUnits',{'mol/m^3','K','K'},...
               'OutputNames',{'Reactor Concentration','Reactor Temperature'},...
               'OutputUnits',{'mol/m^3','K'});
           
%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp,[],mdist);           

%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink')
plot(MPC1,simresult,'detail');
