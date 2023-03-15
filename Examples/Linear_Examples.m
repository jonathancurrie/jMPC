%% jMPC Toolbox Model Predictive Control Examples
% Linear Version

% A collection of example setups for learning the jMPC object. 
% Models are named according to author.

%   Jonathan Currie (C)
%   Control Engineering 2011

%% Oscillatory SISO Example
% Starting with a MATLAB transfer function with an oscillatory response
% this example shows how to build an simulate a simple SISO MPC controller.
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

%% Compare Simulation Methods
simresultML = sim(MPC1,simopts,'matlab');
simresultMX = sim(MPC1,simopts,'mex');
compare(MPC1,simresultML,simresultMX)

%% Rossiter MIMO Model
% Ref: J Rossiter (2004), Model-Based Predictive Control, A Practical
% Approach, CRC Press (Accompanying Software Example).

% A simple MIMO example detailing how to implement soft constraints &
% mismatching initial plant and model states.
clc
clear
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
Ts = 0.5;
%Build jSS Object
Gss = jSS(A,B,C,D);
%Convert to discrete
Plant = c2d(Gss,Ts);
%MPC Model
Model = Plant;

%Horizons & Time
Np = 10;
Nc = 5;
T = 100;
%Setpoint
setp = ones(T,2);
setp(40:end,1) = 1.5;
setp(70:end,2) = 0.5;
%Constraints
con = [];
con.y = [-Inf   1.6;      %out1 [ymin ymax]
         -Inf   1.6];   %out2  
con.slack = [1e3 1e3]; %Soft Output Constraints
%Weights
uwt = [1 1]';
ywt = [1 1]';
% Initial values
Model.x0 = [0.1 0 0.1]'; %Mismatch initial states
%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult);

%% Maciejowski Cessna Model
% Ref: J Maciejowski (2002), Predictive Control with Constraints, Prentice
% Hall, page 64.

% An intermediate example illustrating control of a non-square system (1
% input, 3 outputs). Additional features include naming of I/O, simulating
% in Simulink & using the 'detail' plot feature.
clc
clear
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
%Convert to discrete
Ts = 0.5;
Plant = c2d(Plant,Ts);
%MPC Model
Model = Plant;

%Horizons & Time
Np = 10;
Nc = 3;
T = 40;
%Setpoint (Pitch, Altitude, Altitude Rate)
setp = zeros(T,3);
setp(:,2) = 400*ones(T,1);
%Constraints
con.u = [-0.2618  0.2618  0.5236];   %limit u to +- 15deg and rate to 30
con.y = [-0.35 0.35                  %limit pitch angle to +- 20deg
         -inf inf;
         -inf 30];
%Weights
uwt = 1;
ywt = [1 1 1]';
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Elevator Angle'},...
               'InputUnits',{'rad'},...
               'OutputNames',{'Pitch','Altitude','Altitude Rate'},...
               'OutputUnits',{'rad','m','m/s'});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts,'Simulink');
plot(MPC1,simresult,'timing');

%% MPCTools 3-DOF Helicopter
% Ref: J. �kesson (2006) MPCtools 1.0 Reference Manual, Lund Institute of
% Technology, page 24. Also similar to Quanser 3-DOF Helicopter except
% reordered states.

% A more complex example using a linearized model of a helicopter with fast
% system dynamics. A new feature enabling uncontrolled, but constrained
% outputs is also demonstrated via the use of a ywt = 0 on the pitch axis.
% Also enabled is setpoint look ahead for acausal behaviour as well as we 
% are using the MEX jMPC Engine for typically faster simulation.

% Model Linearized about x0 = [0 0 0], u0 = [0.5 0.5] (see nonlinear
% examples). Therefore x0 = x so our states are true states.
clc
clear
%Linearized Continuous Time Helicopter Model
A =     [0        1        0        0        0        0
         0        0        0        0        0        0
         0        0        0        1        0        0
         0        0        0        0  -0.3626        0
         0        0        0        0        0        1
         0        0        0        0        0        0];
     
B =     [0       0
         0.3626  0.3626
         0       0
         0       0
         0       0
         2.431  -2.431];
     
C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
 
D = zeros(3,2);

%Build jSS object
Plant = jSS(A,B,C,D);
%Convert to discrete
Ts = 0.2;
Plant = c2d(Plant,Ts);
%MPC Model
Model = Plant;

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
con = [];
con.y = [-0.48 0.48;       %Limit elevation to +- 27.5 degrees
         -pi    pi;        %Limit rotation to +- 180 degrees
         -1.05 1.05];      %Limit pitch to +- 60 degrees
con.slack = [Inf Inf 5]'; %Soft constraint with heavy penalty on pitch
%Weights
uwt = [1 1]';
ywt = [5 5 0]';
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Motor 1 Input','Motor 2 Input'},...
               'InputUnits',{'V','V'},...
               'OutputNames',{'Elevation Angle','Rotation Angle','Pitch Angle'},...
               'OutputUnits',{'rad','rad','rad'},...
               'LookAhead',1);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'timing');

%% Wood-Berry Column
% Ref: D. I. Wilson (2008), Advanced Control Using MATLAB, or Stabilising
% the Unstabilisable, Industrial Information & Control Centre, page 89.

% A standard chemical engineering model is the Wood-Berry model of a
% distillation column. The below example shows how to create this MIMO
% model using transfer functions, as well as model the I/O deadtime between
% each input and output. Remember for MPC control of a model with deadtime
% you must have a prediction horizon longer than the deadtime!

% An important point is that when the deadtime is built into a state space
% model as extra states that the states may be reordered. You can use the
% 'Map' property of the jSS class to set initial states in the correct
% positions.
clc
clear
%Linearized Continuous Time Model
Gs = tf({12.8, -18.9; 6.6 -19.4},{[16.7 1],[21 1]; [10.9 1],[14.4 1]});
Gs.ioDelayMatrix = [1,3; 7,3]; %Delay Matrix

%Convert to discrete
Ts = 1;
Gd = c2d(Gs,Ts);

%Build jSS object
Plant = jSS(Gd);
%MPC Model
Model = Plant;

%Horizons & Time
Np = 12;
Nc = 5;
T = 30;
% Setpoint (Distillate, Bottoms)
setp = [1 0.5];
%Constraints
con=[];
con.y = [0 1.01;        
         0 0.55];  
%Weights
uwt = [5 5]'; %discourage input usage
ywt = [1 1]';
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Reflux Flowrate','Steam Flowrate'},...
               'OutputNames',{'Distillate Ethanol','Bottoms Ethanol'},...
               'OutputUnits',{'Mole Fraction','Mole Fraction'});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'detail');

%% Quanser Quadruple Tank
% Ref: Quanser (2003) Coupled Tanks User Manual (www.quanser.com) +
% supplied MALTAB lab files for parameters.

% This more advanced problem introduces two new concepts; using linearized
% models and unmeasured outputs. Using a linearized model is typical within
% many industries and results in a model normalized about a certain
% operating point (u and x). This means at the linear model x=0, your true
% states are biased by these operating points. The jMPC object
% automatically handles this for you, minimizing user input and is
% shown below.

% Also introduced is the concept of unmeasured outputs, where by one or
% more plant outputs are unmeasured. This is where MPC really comes into
% its own; not only can you estimate these outputs using a suitable model,
% but you can also control and constrain them if required! The
% SetUnmeasuredOut() function below shows how this is done on the top two
% tanks.

% See nonlinear examples for more details on the linearization of this
% model.
clc
clear
%Linearized Discrete Time Quad Tank Model
A =     [0.8217    0         0.2381    0
         0         0.8213    0         0.2399
         0         0         0.7365    0
         0         0         0         0.7344]; 

B =     [0.2749    0.0704
         0.0703    0.2773
         0         0.4680
         0.4631    0];

C =  eye(4);
Cm = eye(2,4); %Measured output matrix
 
D = [0     0;
     0     0;
     0     0;
     0     0];

%Build Plant
Ts = 3;
Plant = jSS(A,B,Cm,D,Ts,[]); %Note plant only contains measured outputs
%Build Model with Linearization Point
u0 = [7 7]';
x0 = [15.0746 14.9975 6.2152 6.1025]';
Model = jSS(A,B,C,D,Ts,[],u0,x0);
%Set Unmeasured Outputs in model
Model = SetUnmeasuredOut(Model,[3 4]);

%Horizons & Time
Np = 10;
Nc = 3;
T = 250;
% Setpoint (Bottom Left, Bottom Right)
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
%Weights
uwt = [8 8];
ywt = [10 10 0 0]; %don't control top two tanks, but keep constraints
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'Pump 1 Input','Pump 2 Input'},...
               'InputUnits',{'V','V'},...
               'OutputNames',{'Tank 1 (Bottom Left) Level','Tank 2 (Bottom Right) Level','Tank 3 (Top Left) Level','Tank 4 (Top Right) Level'},...
               'OutputUnits',{'cm','cm','cm','cm'},...
               'LookAhead',1);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'detail');



%% Quadcopter (Fixed Yaw)
% Ref: On-board Model Predictive Control of a Quadrotor Helicopter: Design, 
% Implementation, and Experiments (2012), Patrick Bouffard
clc
clear
%Linearized Discrete Time Quad Tank Model
A =     [1 0.025 0.0031 0 0 0 0 0 0 0
        0 1 0.2453 0 0 0 0 0 0 0
        0 0 0.7969 0.0225 0 0 0 0 0 0
        0 0 -1.7976 0.9767 0 0 0 0 0 0
        0 0 0 0 1 0.025 0.0031 0 0 0
        0 0 0 0 0 1 0.2453 0 0 0
        0 0 0 0 0 0 0.7969 0.0225 0 0
        0 0 0 0 0 0 -1.7976 0.9767 0 0
        0 0 0 0 0 0 0 0 1 0.025
        0 0 0 0 0 0 0 0 0 1]; 

B =     [0 0 0
        0 0 0
        0.01 0 0
        0.9921 0 0
        0 0 0
        0 0 0
        0 0.01 0
        0 0.9921 0
        0 0 -0.00021875
        0 0 -0.0175];

C =  [1 0 0 0 0 0 0 0 0 0
        0 0 1 0 0 0 0 0 0 0
        0 0 0 0 1 0 0 0 0 0
        0 0 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 0 0 1 0];
Cm = eye(2,4); %Measured output matrix
 
D = zeros(3,5);

%Build Plant
Ts = 0.025;
Plant = jSS(A,B,C,D,Ts);
Model = Plant;

%Horizons & Time
Np = 30;
Nc = [5 5 20];
T = 650;
% Setpoint (x1,theta1,x2,theta2,x3)
setp = 0*ones(T,3); setp(1:end,3) = 1;
setp(100:300,1) = 1;
setp(200:400,2) = -1;
setp(500:end,3) = 1.5;
% setp(100:end,1:2) = 20; 
% setp(200:end,2) = 15;
%Constraints
con.u = [-12 12 Inf;   
         -12 12 Inf;
         -12 12 Inf];   
% con.y = [0 25;
%          0 25;
%          0 10;
%          0 10];    
%Weights
uwt = [8 8 8];
ywt = [5 0 5 0 5]; %
%Estimator Gain
Kest = dlqe(Model);

%Set Options
opts = jMPCset('InputNames',{'In1','In2','In3'},...
               'InputUnits',{'','',''},...
               'OutputNames',{'x1','theta1','x2','theta2','x3'},...
               'OutputUnits',{'','','','',''});

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'detail');