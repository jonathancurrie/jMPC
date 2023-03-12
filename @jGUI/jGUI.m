classdef jGUI
%jGUI  Create a GUI Data Object
%
%   jGUIobj = jGUI(Plant,Model,weights,cons,Np,Nc,Ts,state_est,setp,opts)
%   creates a jGUI object which contains the neccesary data to re-run a GUI
%   simulation. The arguments are as follows:
%
%       Plant       - jSS or jNL Model
%       Model       - jSS Model
%       weights     - Structure containing the weighting vectors. Structure
%                     fields are .uwt for input weights and .ywt for output
%                     weights. 
%
%                       Input Weights have the form:
%                           [u1; u2; ...]
%
%                       Output Weights have the form:
%                           [y1; y2; ...]
%
%       cons        - Structure containing the constraints. Structure
%                     fields are .u for input constraints and .y for 
%                     output constraints. 
%
%                       Input constraints have the form:
%                           [u1min u1max delu1max; u2min u2max ....]
%
%                       Output constraints have the form:
%                           [y1min y1max; y2min y2max ...]
%
%       Np          - Prediction Horizon
%       Nc          - Control Horizon
%       Ts          - Controller Sampling Period
%       state_est   - Structure containing state estimation options.
%                     Structure fields are .En to enable state estimation,
%                     and .K for a scalar estimation gain
%       setp        - Initial setpoint vector (row vector)
%       opts        - Structure containing the GUI options. Fields are:
%                       
%                       .window     - Selects the window size (1-6)
%                       .qp         - Selects the QP solver (1-5)
%                       .soft       - Enables / Disables soft constraints
%                       .notes      - Optional user field to describe the
%                                     simulation (string)
%
%   See also jMPC jSS jNL

%   Jonathan Currie (C)
%   AUT University 2009
    
    properties
        Plant;          % Simulation Plant
        Model;          % MPC Model
        Weights;        % Structure containing weighting vectors
        Constraints;    % Structure containing input & output constraints
        Np;             % Prediction Horizon
        Nc;             % Control Horizon
        Ts;             % Controller Sampling Period
        State_Est;      % State Estimation Options
        Setpoints;      % Initial Setpoints
        Options;        % GUI options
    end
    
    %Class Only Methods
    methods
        %Constructor
        function J = jGUI(Plant,Model,Weights,Constraints,Np,Nc,Ts,State_Est,Setpoints,Options)
            if(nargin ~= 10)
                error('This Class only accepts 10 input arguments');
            end
            %Assign Properties
            J.Plant = Plant;
            J.Model = Model;
            J.Weights = Weights;
            J.Constraints = Constraints;
            J.Np = Np;
            J.Nc = Nc;
            J.Ts = Ts;
            J.State_Est = State_Est;
            J.Setpoints = Setpoints;
            J.Options = Options;
        end
        
        %Read MPC from GUI Object
        function MPC1 = read_mpc(J)
            if(J.State_Est.En)
                %Create Estimation Gain
                [r,c] = size(J.Model.A);
                Q = diag(J.State_Est.K.*ones(max([r c]),1));
                Q = Q(1:r,1:c);
                Kest = dlqr(J.Model.A',J.Model.C',Q,eye(size(J.Model.C,1)))';
            else
                Kest= [];
            end
            if(isfield(J.Options,'qp'))
                opts = jMPCset('Solver',J.Options.qp-1);
            else
                opts = jMPCset();
            end
            %Re-Create MPC Controller
            MPC1 = jMPC(J.Model,J.Np,J.Nc,J.Weights.uwt,J.Weights.ywt,...
                J.Constraints,Kest,opts);
        end
    end
end
        