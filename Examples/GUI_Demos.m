%% Demonstration Models

% To use these demonstration models:

% - run jMPCtool
% - run the cell with the demonstration you wish to run
% - in jMPC simulation tool, file->load->import from workspace
% - select the example number from the list

%% 1 - Minimise Use of Input 2
clear
%Plant
G1p = tf({[1 -1],[1 2]},{[1 1],[1 4 5]});
%Model
G1m = tf({[1.2 -1.1],[1.5 2.2]},{[1.4 1.7],[1.2 4.4 5.1]});

Ts = 0.1;
Np = 10;
Nc = 5;
setp = 1;                   %[setp1]
Constraints.u = [-5 5 2.5;  %[umin umax del_umax]
                 -5 5 2.5];
Constraints.y = [-5 5];     %[ymin ymax] 
Weights.uwt = [0.1 0.2]';   %[in1 in2]
Weights.ywt = [0.1]';       %[out1]
State_Est.K = 0.8;
State_Est.En = 1;
Options.window = 4;
Options.notes = sprintf('2 Input 1 Output (MISO) System\n\nIn this example we wish to minimise the use of input 2 while maintaining an acceptable response. Includes Plant-Model Mismatch & normalised constraints added.');
Example1 = jGUI(G1p,G1m,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);

%% 2 - Minimise Use of Output 2
clear
%Plant
G1p = tf({[1 -1],[1 2],[0.2 1];[0.5 -2],[0.6 2.7],[0.3 1.4]},{[1 1],[1 4 5],[1.5 2.2];[1 2],[1 2 5],[1.1 2.7]});
%Model
G1m = tf({[1.2 -1.1],[0.3 1],[0.2 1];[0.5 -2],[0.6 2.7],[0.3 1.4]},{[1.4 1.7],[1.2 4.4 5.1],[1.2 1.2];[1 2],[1 2 5],[1.1 2.7]});

Ts = 0.1;
Np = 15;
Nc = 10;
setp = [1 -1];
Constraints.u = [-15 15 1;
                 -Inf Inf 1e6;
                 -Inf Inf 1e6];
Constraints.y = [-5 5;
                 -5 5];      
Weights.uwt = [2 2 2]';
Weights.ywt = [1 2]';
State_Est.K = 0.8;
State_Est.En = 1;
Options.window = 4;
Options.notes = sprintf('3 Input 2 Output MIMO System\n\nStrong State Interactions\n\nIn this example we wish to minimise the interaction of output1 on output2');
Example2 = jGUI(G1p,G1m,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);

%% 3 - Mismatched with Dead Time (WARNING DISABLE AUTO TS CALCULATION!)
clear
%Plant
G1p = tf({[1 -1],[1 2]},{[1 1],[1 4 5]},'iodelayMatrix',[4.1 5]);
%Model
G1m = tf({[1.2 -1.1],[1.5 2.2]},{[1.4 1.7],[1.2 4.4 5.1]},'iodelayMatrix',[4.2 5.4]);

Ts = 0.5;
Np = 30;
Nc = 20;
setp = 1;
Constraints.u = [-5 inf 2.5;
                 -5 inf 2.5];
Constraints.y = [-5 5];      
Weights.uwt = [1 1]';
Weights.ywt = [1]';
State_Est.K = 0.8;
State_Est.En = 1;
Options.window = 5;
Options.notes = sprintf('2 Input 1 Output (MISO) System\n\nIncludes Plant-Model Mismatch + Mismatched Dead Time.');
Example3 = jGUI(G1p,G1m,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);

%% 4 Internet SS Model
clear
%Jet Transport Aircraft 0.8Mach 40,000ft
A = [-0.0558   -0.9968    0.0802    0.0415
      0.5980   -0.1150   -0.0318         0
     -3.0500    0.3880   -0.4650         0
           0    0.0805    1.0000         0];
B = [ 0.0073         0
     -0.4750    0.0077
      0.1530    0.1430
           0         0];
C = [0     1     0     0
     0     0     0     1];
D = [0     0
     0     0];

states = {'beta' 'yaw' 'roll' 'phi'};
inputs = {'rudder' 'aileron'};
outputs = {'yaw rate' 'bank angle'};

sys = ss(A,B,C,D,'statename',states,...
                 'inputname',inputs,...
                 'outputname',outputs);
                 
Plant = jSS(sys);
Plant.x0 = [0 0 0 0]';
Model = Plant;
Ts = 0.33;
Np = 10;
Nc = 5;
setp = [0 0.2];
Constraints.u = [-5 5 2.5;
                 -5 5 2.5];
Constraints.y = [-10 10;
                 -10 10];      
Weights.uwt = [1 1]';
Weights.ywt = [1 1]';
State_Est.K = 0.8;
State_Est.En = 1;
Options.window = 5;
Options.notes = sprintf('Linearised Jet Airliner\n0.8MACH 40,000ft\n Inputs:\n1-Rudder\n2-Aileron\n\nOutputs\n1-Yaw Rate\n2-Bank Angle');
Example4 = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);