function Example = gui_examples(index)
% Example Setups for the GUI

% Jonathan Currie (C)
% Control Engineering 2009

switch(index)
    %Rossiter SISO
    case 1
        Gs = tf(2,[0.7 0.2 1]);
        Ts = 0.1;
        Gd = c2d(Gs,Ts);
        Model = jSS(Gd);
        Plant = Model;
        Np = 10;
        Nc = 5;
        setp = 1;
        Constraints.u = [-inf   inf    0.2];  
        Constraints.y = [-inf   inf];  
        Weights.uwt = 0.5;
        Weights.ywt = 0.5;
        Plant.x0 = [0 0]';
        Model.x0 = [0 0]';
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 4;
        Options.notes = sprintf('Rossiter SISO System\nNp = 10, Nc = 5\nInput Rate Limited\n\nNo Model-Plant Mismatch');
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
    %Rossier MIMO
    case 2
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
        Model = jSS(A,B,C,D,Ts);
        Plant = Model;
        Np = 10;
        Nc = 5;
        setp = [1 1];
        Constraints.u = [-2.4   2.7     0.5;   %in1 umin umax delumax
                         -0.5   2.3     0.6];  %in2   
        Constraints.y = [-1.5   1.7;      %out1 ymin ymax
                         -1.4   1.6];     %out2   
        Weights.uwt = [1 1]';
        Weights.ywt = [1 1]';
        Plant.x0 = [0 0 0]';
        Model.x0 = [0 0.2 0]';
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 4;
        Options.notes = sprintf('Rossiter MIMO System\nNp = 10, Nc = 5\nFully Constrained\n\nNo Model-Plant Mismatch');
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
    %Maciejowski Linearized Continuous Time Cessna Model
    case 3
        A = [-1.2822   0  0.98   0;
                 0     0   1     0;
             -5.4293   0 -1.8366 0;
             -128.2 128.2  0     0];
        B = [-0.3; 0; -17; 0];
        C = [   0      1  0 0;
                0      0  0 1;
             -128.2 128.2 0 0];
        D = zeros(3,1);
        Plant = jSS(A,B,C,D);
        Ts = 0.5;
        Plant = c2d(Plant,Ts);
        Model = Plant;
        Np = 10;
        Nc = 5;
        setp = [0 10 0];
        Constraints.u = [-0.2618  0.2618  0.5236];   %limit u to +- 15deg and rate to 30
        Constraints.y = [-0.35 0.35                  %limit pitch angle to +- 20deg
                           -inf inf;
                           -inf 30];
        Weights.uwt = 1;
        Weights.ywt = [1 1 1]';
        Plant.x0 = [0 0 0 0]';
        Model.x0 = [0 0 0 0]';
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 5;
        Options.notes = sprintf('Maciejowski Linearised Cessna Model\nNp = 10, Nc = 5\nLimit Elevator to +- 15deg\nLimit Elevator Rate to +-30deg\nLimit pitch angle to +-20deg\n\nNo Model-Plant Mismatch\nOut1 Pitch\nOut2 Altitude Rate\nOut3 Altitude\nIn1 Elevator');
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
    %Quanser 3DOF Linearized Continuous Time Helicopter Model
    case 4
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
        Plant = jSS(A,B,C,D);
        Ts = 0.1;
        Plant = c2d(Plant,Ts);
        Model = Plant; %no model plant mismatch
        Np = 15;
        Nc = 6;
        setp = [20*pi/180 0 50*pi/180];
        Constraints.u = [-24 24 15;   %Limit of UPM is +-24V, rate unsure
                         -24 24 15];
        Constraints.y = [-0.48 0.48;        %Limit elev to +- 27.5 degrees
                         -1.05 1.05;       %Limit pitch to +- 60 degrees
                         -pi    pi];        %Limit travel to +- 180 degrees
        Weights.uwt = [0.05 0.05]';
        Weights.ywt = [100 10 100]';
        Plant.x0 = [0 0 0 0 0 0]';
        Model.x0 = [0 0 0 0 0 0]';
        opts.window = 4;
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 4;
        Options.notes = sprintf('Quanser 3DOF Helicopter\nNp = 15, Nc = 6\nLimit Input to +-24V\nLimit Elevation to +-27.5deg\nLimit Pitch to +-60deg\nLimit Rotation to +-180deg\n\nNo Model-Plant Mismatch\nOut1 Elevation\nOut2 Pitch\nOut3 Rotation\nIn1 F Motor V\nIn2 B Motor V');
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
    %DIW Wood Berry
    case 5
        Gs = tf({12.8, -18.9; 6.6 -19.4}, ...
        { [16.7 1],[21 1]; [10.9 1],[14.4 1]}, ...
        'ioDelayMatrix',[1,3; 7,3], ...
        'InputName',{'Reflux','Steam'}, ...
        'OutputName',{'Distillate','bottoms'});
        Model = Gs;
        Plant = Model;
        Ts = 1;
        Np = 15;
        Nc = 5;
        setp = [1 0.5];
        Constraints.u = [-5 5 1;   
                 -5 5 1];
        Constraints.y = [-5 5;        
                 -5 5];            
        Weights.uwt = [10 10]';
        Weights.ywt = [1 1]';

        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 5;
        Options.notes = sprintf('DIW Wood Berry Model\nNp = 10, Nc = 5\nFully Constrained\n\nNo Model-Plant Mismatch\n Dead Time Included\nOut1 Distillate\nOut2 Bottoms\nIn1 Reflux\nIn2 Steam');
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
        
    %Lund Helicopter
    case 6
        %Parameters
        Je = 0.91;
        la = 0.66;
        Kf = 0.5;
        Fg = 0.5;
        Tg = la*Fg;
        Jp = 0.0364;
        lh = 0.177;
        Jt = 0.91;

        % Linear Model
        A = [0 1 0 0 0 0;
             0 0 0 0 0 0;
             0 0 0 1 0 0;
             0 0 0 0 -Fg*la/Jt 0;     
             0 0 0 0 0 1;
             0 0 0 0 0 0;
             ];

        B = [0 0;
             Kf*la/Je Kf*la/Je;
             0 0;
             0 0;
             0 0;
             Kf*lh/Jp -Kf*lh/Jp];

        C = [1 0 0 0 0 0;
             0 0 1 0 0 0;
             0 0 0 0 1 0];

        D = zeros(3,2);

        %Build jSS object
        Model = jSS(A,B,C,D);
        %Discretize model
        Ts = 0.12;
        Model = c2d(Model,Ts);
        %Non Linear Plant
        param = {Je,la,Kf,Fg,Tg,Jp,lh,Jt}; %parameter cell array
        Plant = jNL(@nl_heli,C,param);
        
        Np = 30;
        Nc = 10;
        setp = [20*pi/180 50*pi/180 0];
        Constraints.u = [-3 3 Inf;   
                         -3 3 Inf];
        Constraints.y = [-5    0.6;       
                         -pi   pi       
                         -1    1];      
        %Weighting
        Weights.uwt = [1 1]';
        Weights.ywt = [10 10 1]'; 
        
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 4;
        Options.notes = sprintf('Lund NL Helicopter\nNp = 30, Nc = 10\n\nNon-Linear Plant\nOut1 = Elevation\nOut2 = Rotation\nOut3 = Pitch\nIn1 F Motor V\nIn2 B Motor V');
        
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);

    %DIW Inverted Pendulum        
    case 7
        %Parameters
        M = 0.455;
        m = 0.21;
        l = 0.5*0.61;
        g = 9.81;

        %Linear Model
        A = [0   1   0             0;
             0   0   -g*m/M        0;
             0   0   0             1;
             0   0   g*(M+m)/(l*M) 0];

        B = [    0;
                1/M;
                 0;
             -1/(l*M)];

        C = [1 0 0 0;
             0 0 1 0];

        D = 0;

        %Build jSS object
        Model = jSS(A,B,C,D);
        %Discretize model
        Ts = 0.05;
        Model = c2d(Model,Ts);
        %Non Linear Plant
        param = {M,m,l,g}; %parameter cell array
        Plant = jNL(@nl_pend,C,param);

        Np = 40;
        Nc = 10;
        setp = [1 0];
        Constraints.u = [-10 10 10];
        Constraints.y = [-2    2;       
                        -pi/4 pi/4];      
        Weights.uwt = 2;
        Weights.ywt = [2 1]';
        
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 4;
        Options.notes = sprintf('DIW NL Inverted Pendulum\nNp = 40, Nc = 10\n\nNon-Linear Plant\nOut1 = Cart Position\nOut2 = Pendulum Angle\nIn1 Cart Force');
        
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);

    %Lund Quadruple Tank        
    case 8
        %Parameters
        A1  = 15.5179; %tank cross sectional area
        A2  = 15.5179;
        A3  = 15.5179;
        A4  = 15.5179;
        a1  = 0.1781; %outlet cross sectional area
        a2  = 0.1781;
        a3  = 0.1781;
        a4  = 0.1781;
        g   = 981; 
        k1  = 4.35; % pump coefficients
        k2  = 4.39; 
        g1  = 0.36; % ratio of allocated pump capacity between lower and upper tank
        g2  = 0.36;

        C =  eye(4); %measure all states

        %Non Linear Plant
        param = {A1,A2,A3,A4,a1,a2,a3,a4,g,g1,g2,k1,k2}; %parameter cell array
        Plant = jNL(@nl_quadtank,C,param);

        %Linearize Plant
        Model = linearize(Plant,[7 7]);
        %Build jSS object
        Model = jSS(Model);
        %Discretize model
        Ts = 3;
        Model = c2d(Model,Ts);

        Np = 10;
        Nc = 3;
        setp = [14 19];
        Constraints.u = [0 12 Inf;   
                         0 12 Inf];
        Constraints.y = [0 25;
                         0 25;
                         0 25
                         0 25];      
        Weights.uwt = [8 8]';
        Weights.ywt = [10 10 0 0]';
        
        State_Est.K = 0.8;
        State_Est.En = 1;
        Options.window = 5;
        Options.notes = sprintf('Quanser NL Quad Tank\nNp = 10, Nc = 3\n\nNon-Linear Plant\nOut1-4 = Tank Level 1-4\nIn1-2 = Pump 1-2 V\nObjective To Control lower 2 Tanks (1,2) Level');
        
        Example = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,setp,Options);
    
    otherwise
        Example.Options.notes = [];
end