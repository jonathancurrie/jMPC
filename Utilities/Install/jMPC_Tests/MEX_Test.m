%% MEX TESTING
% Developer use only...

%% Rossiter SISO
clc
clear
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
setp = ones(T+1,1);
setp(75:150) = 0.5;
%Disturbances
udist = zeros(T+1,1);
udist(60:120) = -0.3;
ydist = rand(T+1,1)/50;

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
opts.solver = 2;
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);%,udist,ydist);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);

%% Rossiter MIMO Model
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
setp = ones(T+1,2);

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
opts.solver = 2;
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);

%% Maciejowski Cessna Model
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
setp = zeros(T+1,3);
setp(:,2) = 400*ones(T+1,1);
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
opts.solver = 2;
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);

%% Quanser 3-DOF Helicopter
clc
clear
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
setp = zeros(T+1,3);
setp(:,1) = 20*pi/180*ones(T+1,1);
setp(50:100,1) = -20*pi/180;
setp(101:150,1) = 5*pi/180;
setp(151:200,1) = -15*pi/180;
setp(100:160,3) = 100*pi/180;
setp(161:end,3) = -100*pi/180;
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
opts.solver = 2;
opts.look_ahead = 0;
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);


%% DIW Wood Berry
clc
clear

Gs = tf({12.8, -18.9; 6.6 -19.4}, ...
{ [16.7 1],[21 1]; [10.9 1],[14.4 1]}, ...
'InputName',{'Reflux','Steam'}, ...
'OutputName',{'Distillate','bottoms'});
Gs.ioDelayMatrix = [1,3; 7,3];

Ts = 1;
Gd = c2d(Gs,Ts);
Model = jSS(Gd);
Plant = Model;

Np = 10;
Nc = 5;
T = 30;
setp = [1 0.5];
con.u = [-5 5 1;   
         -5 5 1];
con.y = [-2 3;        
         -1 2];      
uwt = [10 10]';
ywt = [1 1]';

%Estimator Gain
Kest = dlqe(Model);

%-- Build MPC & Simulation --%
opts.solver = 2;
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);


%% MPC Tools Coupled Tanks
clc
clear
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
 
D = [0     0;
     0     0;
     0     0;
     0     0];

%Build jSS object
Ts = 1;
Model = jSS(A,B,C,D,Ts);
%Set Linearization Points
u_op = [7 7]';
x_op = [15.0684 14.9969 6.2123 6.0976]';
y_op = [15.0684 14.9969 6.2123 6.0976]';
Model = Model.SetLinearization(u_op,x_op,y_op);

%Horizons & Time
Np = 40;
Nc = 10;
T = 300;
% Setpoint
setp = 15*ones(T+1,2);
setp(100:end,1:2) = 20; 
setp(200:end,2) = 15;

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
Kc = eye(2,4); %remove measurements from top two tanks
Plant = jSS(Model.A,Model.B,Kc,Model.D,Ts);
Kest = dlqe(Plant.A,eye(size(Plant.A)),Plant.C,diag([1,1,15,15]),diag([1 1]));

%-- Build MPC & Simulation --%
opts.solver = 2;
opts.look_ahead = 0;
opts.meas_out = [1 2]; %indicate which outputs are measured (bottom only)

%Build
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
simopts = jSIM(MPC1,Plant,T,setp);%,udist,ydist);

%-- Simulate & Plot Result --%
sim1 = sim(MPC1,simopts,'MEX');
sim2 = sim(MPC1,simopts,'Simulink');
compare(MPC1,sim1,sim2);