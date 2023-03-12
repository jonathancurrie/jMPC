%% Generate Embedded MPC Controller for the Quanser 3-DOF Helicopter
% May 2014
clc
clear all
%% Build MPC Simulation
% The code below generates a nonlinear model and a linearized model
% suitable for controller generation. It then simulates the linear model
% against the nonlinear one to validate the controller tuning.
clc
%Get Nonlinear and Linear Models + Parameters
[Gs, Vop, nlheli, param] = setup_embed_heli();
%Convert Linear Model to Discrete
Ts = 0.03; %30ms
Model = jSS(c2d(Gs,Ts));
%MPC Model
x_op = zeros(6,1);
Model = Model.SetLinearization(Vop,x_op); 
Model.x0 = x_op;

%Build Plant Model
Plant = jNL(nlheli,Model.C,param);

%Horizons & Time
Np = 80;
Nc = [5 5];
T = 1200;
%Setpoint (Elevation, Pitch, Rotation)
setp = zeros(T,2);
setp(50:end,1) = 0.2618;
setp(300:end,2) = 1.5472;
setp(800:end,2) = -1.0472;
%Constraints
con=[]; pitch = 30*pi/180;
con.u = [-20 20 Inf;
         -20 20 Inf];
con.y = [-Inf   Inf;      
         -pitch pitch;                    
         -Inf   Inf];    
con.slack = [Inf 1000 Inf];
     
%Weights
uwt = [1 1]';
ywt = [12 0 5]'; %[elev pit rot]
%Estimator Gain
Q = 2*eye(6); R = 0.1*eye(3); Q(4,4) = 1; Q(5,5) = 1; Q(6,6) = 0.1;
Kest = dlqe(Model,Q,R);

opts = jMPCset('Single',1,'QPTol',1e-4,'InitialU',Vop);

%-- Build MPC & Simulation --%
MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
simopts = jSIM(MPC1,Plant,T,setp);
%Add serial port connected to 3-DOF acquisition system
sd = serial('COM6','BaudRate',1250000);
simopts.opts.serialdevice = sd;
%-- Simulate & Plot Result --%
simresult = sim(MPC1,simopts)
plot(MPC1,simresult,'timing');


%% Embed The Controller (assumes we are in Matlab\jMPC\Examples\Quanser 3DOF Helicopter\ folder)
eopts = jMPCeset('arch','c2000','accelqp',1,'dir','..\..\..\Testing\EmbedMPC\');
embed(MPC1,simopts,eopts);


%% PIL Verification [USE TI Project Embedded_jMPC]
% Run the controller in PIL mode to verify the controller on the TI IC.
% IMPORTANT: Start the controller FIRST before running this code.
simhelipil = sim(MPC1,simopts,'PIL')
plot(MPC1,simhelipil,'timing');


%% Run Embedded Controller and Record [USE TI Project HeliMPC]
% IMPORTANT: Start the controller FIRST before running this code.
clc
fclose(sd);

%Number of samples to collect (time = n*Ts)
n = 5000;

%Preallocate
u = zeros(2,n);
xm = zeros(3,n);
ym = zeros(3,n);
yp = zeros(3,n);
setp = zeros(3,n);
time = zeros(n,1);
iter = zeros(n,1);
status = zeros(n,1);

%Open Serial Port
if(strcmp(sd.Status,'closed'))
    fopen(sd);
end

%Send Getting Started Byte
jMPC_SerialTx(sd,uint8(1));

%Read in runtime variables
for i = 1:n
    u(:,i) = jMPC_SerialRx(sd,2,'float');
    xm(:,i) = jMPC_SerialRx(sd,3,'float');
    ym(:,i) = jMPC_SerialRx(sd,3,'float');
    yp(:,i) = jMPC_SerialRx(sd,3,'float');
    setp(:,i) = jMPC_SerialRx(sd,3,'float');
    time(i) = jMPC_SerialRx(sd,1,'uint32');
    iter(i) = jMPC_SerialRx(sd,1,'int8');
    status(i) = jMPC_SerialRx(sd,1,'int8');    
end
fclose(sd);

%% Optionally Save Data
% save helimpc u xm ym yp setp time iter status MPC1 simopts


%% Plot Results
Ts = MPC1.Model.Ts;
T = n;%simopts.T;
pitch = abs(MPC1.constraints.y(2,1));
k = (0:T-1)*Ts; 
idx = true(T,1);
clf;
subplot(321);
plot(k,yp(1,idx)*180/pi,'r'); hold on; plot(k,setp(1,idx)*180/pi,'--','color',jcolour('spgrey')); hold off;
set(gca,'XTickLabel',[],'ylim',[0 18],'xlim',[0 (T-1)*Ts]); 
ylabel('Elevation [\circ]'); 

subplot(322);
plot(k,yp(2,idx)*180/pi,'r');
h = hline([pitch*180/pi;-pitch*180/pi],':'); set(h,'color',jcolour('softgrey'));
set(gca,'XTickLabel',[],'ylim',[-(pitch)*180/pi-20 (pitch)*180/pi+20],'xlim',[0 (T-1)*Ts]);
ylabel('Pitch [\circ]');

subplot(323);
plot(k,yp(3,idx)*180/pi,'r'); hold on; plot(k,setp(2,idx)*180/pi,'--','color',jcolour('spgrey')); hold off;
% h = hline(-1.1*180/pi,'r:'); set(h,'color',jcolour('softgrey'));
set(gca,'XTickLabel',[],'ylim',[-50 230],'xlim',[0 (T-1)*Ts]);
ylabel('Rotation [\circ]'); 

subplot(324);
stairs(k,u(:,idx)');
hline(20,'b:');
set(gca,'XTickLabel',[],'ylim',[-10 24],'xlim',[0 (T-1)*Ts]);
ylabel('Motor Voltage [V]'); 

subplot(325);
stairs(k,iter(idx),'color',[0.7 0.3 0.2]);
set(gca,'ylim',[0 12],'xlim',[0 (T-1)*Ts]);
text(17,11,sprintf('Max %d',max(iter)));
ylabel('QP Iterations'); xlabel('Simulation Time [s]');

subplot(326); time(1) = 0;
stairs(k,time(idx)/1e3,'m');
set(gca,'ylim',[0 20],'xlim',[0 (T-1)*Ts]); 
text(13,17,sprintf('Max %1.2fms',max(time/1e3)));
ylabel('Execution Time [ms]'); xlabel('Simulation Time [s]');

axes('Position',[0 0 1 1],'Visible','off','Tag','CommonTitle');
text(0.3,0.98,'Embedded MPC Control of the 3DOF Helicopter')

squeezefigs(3,3,-0.1,[],false)
boldfigs(2,13)
set(gcf,'position',jsize('32'));
% printplot('../figs/embedmpc/helimpcsp');
