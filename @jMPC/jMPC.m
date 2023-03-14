classdef jMPC
%jMPC Create a Model Predictive Controller Object
%
%   jMPCobj = jMPC(Model,Np,Nc,uwt,ywt) creates a jMPC object with the
%   minimum required information for a controller. Model is a jSS object
%   which is the internal Model used by the controller for prediction. Np
%   and Nc are the prediction and control horizons, respectively. Nc can
%   also represent the blocking moves across the prediction horizon. uwt 
%   and ywt are column vectors containing the tuning weights for the
%   controller.
%
%   jMPCobj = jMPC(Model,Np,Nc,uwt,ywt,con) specifies the controller
%   constraints. Structure fields are .u for input constraints and .y for 
%   output constraints. 
%
%       Input constraints have the form:
%           [u1min u1max delu1max; u2min u2max delu2max; ...]
%
%       Output constraints have the form:
%           [y1min y1max; y2min y2max ...]
%
%   When con is omitted (or empty) no constraints are present, and the 
%   controller is purely unconstrained. You can also use +-inf to leave 
%   individual inputs/outputs of the constraint vectors unconstrained. By 
%   default the output constraints are hard (controller will enforce unless 
%   infeasible). To enable soft constraints add .slack field to the con 
%   structure with a weight for each output between 1 and 1e3. Larger 
%   weights penalize the constraint more.
%
%   jMPCobj = jMPC(Model,...,con,Kest) specifies the state estimation gain
%   matrix, Kest, for the controller. It is assumed no states are directly
%   measureable. When this argument is omitted (or empty) no state
%   estimation is used.
%
%   jMPCobj = jMPC(Model,...,Kest,opts) specifies options for the
%   controller, as a structure. The fields are set using 'jMPCset'.
%
%   See also jMPC.sim jMPC.plot jMPC.mpcsolve jSIM jSS jNL jGUI ScrollPlot
    
%   Jonathan Currie (C)
%   Control Engineering 2011-2023

    properties (SetAccess = private)
        Model           % Augmented jSS Model       
        Np              % Prediction Horizon
        Nc              % Control Horizon (Or Blocking Moves)
        Nb              % Number of Blocking Moves
        QP              % QP Structure
        constraints     % Constraints structure
        state_est       % State Estimation Structure        
        pred            % Prediction Matrix Structure
        lin             % Linearization structure
        initial         % Initial u,x,y        
        index           % Structure of Index vectors
        mpcopts         % Controller Options        
        sizes           % Size structure (n_in, n_out, states, etc)        
        StdModel        % Original Model
    end
    
    %Class Only Methods
    methods
        %-- Constructor --%
        function J = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
            %Check for option update interface
            if(nargin == 2 && isa(Model,'jMPC') && isstruct(Np))
                Jold = Model; opts = Np;
                Model = Jold.StdModel;
                Np = Jold.Np; Nc = Jold.Nc;
                uwt = Jold.QP.uwt; ywt = Jold.QP.ywt;
                con = struct('u',Jold.constraints.u,'y',Jold.constraints.y,'slack',Jold.constraints.slack);
                Kest = Jold.state_est.StdKest;
            else
                %Check Optional Inputs
                if(nargin < 8 || isempty(opts)), opts = jMPCset; else opts = jMPCset(opts); end
                if(nargin < 7), Kest = []; end
                if(nargin < 6), con = []; end
            end
            %Check User Inputs
            opts = jMPC.argcheckMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
            %Build MPC
            MPCstruct = jMPC.buildJMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
            %Allocate Class Properties
            J.Model = MPCstruct.Model;
            J.StdModel = Model;
            J.sizes = MPCstruct.sizes;
            J.constraints = MPCstruct.constraints;
            J.QP = MPCstruct.QP;
            J.state_est = MPCstruct.state_est;
            J.pred = MPCstruct.pred;
            J.lin = MPCstruct.lin;
            J.initial = MPCstruct.initial;
            J.index = MPCstruct.index;
            J.mpcopts = MPCstruct.opts;
            J.Np = MPCstruct.Np; J.Nc = MPCstruct.Nc; J.Nb = MPCstruct.Nb;
            %If requested as single, convert   
            if(J.mpcopts.Single)
                J = single(J);
            end
        end

        %-- Calculate optimal input --%
        function [del_u,stats,QP] = mpcsolve(J,del_xm,u,setp,mdist,k,tstart,forceQP,saveQP)
            %MPCSOLVE  Solve QP for plant input
            %
            %   [del_u,stats] = mpcsolve(jMPCobj,del_xm,u,setp,mdist,k,tstart,verb) 
            %   calls the selected QP solver to calculate the optimal plant
            %   input. Arguments required:
            %
            %       jMPCobj     - MPC Controller
            %       del_xm      - Model State Increment
            %       u           - Previous Plant Input
            %       setp        - Setpoint vector
            %       mdist       - Measured Disturbance Vector
            %       k           - Simulation step (error display only)
            %       tstart      - Stop Watch Start Time
            %       verb        - Status verbosity
            %
            %   The function returns all calculated delta inputs (Nc long)
            %   and a status structure.
            %            
            %Check inputs
            if(nargin < 9), saveQP = false; end 
            if(nargin < 8 || isempty(forceQP)), forceQP = false; end
            if(nargin < 7 || isempty(tstart)), tstart = tic; end
            if(nargin < 6 || isempty(k)), k = 1; end
            if(nargin < 5 || isempty(mdist))
                if(J.mpcopts.Single)
                    mdist = zeros(1,J.sizes.nm_dist,'single'); 
                else
                    mdist = zeros(1,J.sizes.nm_dist); 
                end
            end
            if(nargin < 4), error('Not enough input arguments!'); end
            %Update Dynamic RHS of QP Problem
            [b,f] = update_rhs(J,del_xm,u,setp,mdist,k); trhs = toc(tstart);                            
            %Solve Constrained Problem
            [del_u,stats,warm] = solve_qp(J,b,f,k,tstart,forceQP); stats.trhs = trhs;
            %Save if required
            if(saveQP)
                QP = struct('b',b,'f',f,'z',warm.z,'lam',warm.lam,'t',warm.t);
            else
                QP = [];
            end
        end
        
        %-- Run Simulation --%
        function simresult = sim(J,simopts,mode)
            %SIM  Simulate a jMPC Controller with supplied Simulation
            %Environment
            %
            %   jSIMobj = sim(jMPCobj,jSIMobj,mode) simulates the supplied
            %   Model Predictive Controller (jMPCobj) within the supplied
            %   Simulation Environment (jSIMobj). There are four modes:
            %
            %       'Matlab'     - Using the class functions
            %       'Simulink'   - Using a Simulink implementation (Beta)
            %       'MPCToolbox' - Using the MATLAB MPC Toolbox (Beta)
            %       'MEX'        - Using a High Speed MEX Implementation
            %
            %
            %   The returned jSIMobj will contain the results of
            %   simulation.
            %
            if(nargin < 3)
                if(ispc && isa(simopts.Plant,'jSS') && (~isempty(strfind(J.mpcopts.QPSolver,'wright')) || ~isempty(strfind(J.mpcopts.QPSolver,'mehrotra'))))
                    mode = 'Mex';
                else
                    mode = 'Matlab';
                end
            end            
            %Simulate on chosen platform
            switch(lower(mode))
                case 'matlab' 
                    if(J.mpcopts.Single), simopts = single(simopts); end
                    if(isa(J.StdModel,'jNL'))
                        simresult = simNLMPC(J,simopts);
                    else
                        simresult = simJMPC(J,simopts);
                    end
                case 'simulink'
                    sk = which('simset');
                    if(isempty(sk)), error('Simulink is not found on your system'); end
                    if(J.mpcopts.LookAhead == 1), error('Simulink cannot model setpoint look ahead!'); end
                    if(isa(J.Model.A,'single')), error('Simulink does not currently simulate single precision jMPC controllers'); end
                    simulinkopts = simset('SrcWorkspace','current'); %Fixed bug here
                    SIMJMPC = J; SIMJMPCPlant = simopts.Plant;  %#ok<*NASGU>
                    if(isa(simopts.Plant,'jNL'))  
                        if(exist('jMPC_simulinkNL','file') ~= 4); error('Please Confirm jMPC_simulinkNL.mdl is on the Matlab Path'); end 
                        mod = 'jMPC_simulinkNL';   
                        switch(simopts.Plant.odesolver)
                            case(0)
                                simulinkopts = simset(simulinkopts,'Solver','ode45');
                            case(1)
                                simulinkopts = simset(simulinkopts,'Solver','ode23s');
                            case(2)                       
                                simulinkopts = simset(simulinkopts,'Solver','ode15s');                            
                        end
                    else
                        if(exist('jMPC_simulink','file') ~= 4); error('Please Confirm jMPC_simulink.mdl is on the Matlab Path'); end 
                        mod = 'jMPC_simulink';
                    end                                                         
                    sim(mod,[0 simopts.simulink.Tfinal],simulinkopts);
                    %Save plot vectors
                    n_in = J.sizes.n_in; nm_in = J.sizes.nm_in; n_out = J.sizes.n_out; nm_out = J.sizes.nm_out; states = J.sizes.states;
                    values = squeeze(SIMJMPCRES.signals.values);    idx1 = 1; idx2 = n_in;
                    plotvec.u = values(:,idx1:idx2);                idx1 = idx2+1; idx2 = idx2+nm_in;
                    plotvec.del_u = values(:,idx1:idx2);            idx1 = idx2+1; idx2 = idx2+nm_out;
                    plotvec.yp = values(:,idx1:idx2);               idx1 = idx2+1; idx2 = idx2+n_out;
                    plotvec.ym = values(:,idx1:idx2);               idx1 = idx2+1; idx2 = idx2+length(simopts.Plant.x0);
                    plotvec.xp = values(:,idx1:idx2);               idx1 = idx2+1; idx2 = idx2+states;
                    plotvec.xm = values(:,idx1:idx2);               idx1 = idx2+1; idx2 = idx2+1;
                    qpstats.iter = [0;values(1:end-1,idx1:idx2)];         idx1 = idx2+1; idx2 = idx2+1;
                    qpstats.status = [0;values(1:end-1,idx1:idx2)];       idx1 = idx2+1; idx2 = idx2+1;
                    simopts.timing.total = [0;values(1:end-1,idx1:idx2)./1e3];     idx1 = idx2+1; idx2 = idx2+1;
                    %Shift Accordingly
                    plotvec.u = [(J.initial.u+J.lin.u_op)'; plotvec.u(1:end-1,:)];
                    %Create Result Object
                    simresult = simopts;
                    simresult.plotvec = plotvec;
                    simresult.qpstats = qpstats;
                    simresult.opts.result = 1;
                    simresult.mode = 'Simulink [Double Precision]';
                case 'mpctoolbox'
                    nMPC = buildMatlabMPC(J,simopts); opts = mpcsimopt(nMPC); opts.StatusBar = 'on';
                    [y,t,u,xp,xmpc] = sim(nMPC,simopts.T+1,simopts.setp,opts); %#ok<ASGLU>
                    %Save plot vectors
                    plotvec.u = [J.initial.u'; u(1:end-1,:)]; plotvec.yp = y; plotvec.xp = xp; plotvec.xm = xmpc.Plant;
                    plotvec.del_u = u-xmpc.LastMove; plotvec.ym = [];
                    %Create Result Object
                    simresult = simopts;
                    simresult.plotvec = plotvec;
                    simresult.qpstats = struct('iter',[],'status',[]);
                    simresult.timing = struct('total',[]);
                    simresult.opts.result = 1;
                    simresult.mode = 'MPC Toolbox [Double Precision]';
                case 'mex'
                    if(isa(simopts.Plant,'jNL')); error('Currently Nonlinear Models cannot be simulated using the MEX jMPCEngine'); end
                    if(J.mpcopts.Single)
                        plotvec = jMPCSEngine(J,single(simopts));
                    else
                        plotvec = jMPCEngine(J,simopts);
                    end
                    %Create Result Object
                    simresult = simopts;
                    simresult.plotvec = plotvec; 
                    simresult.qpstats = struct('iter',plotvec.qpiter,'status',plotvec.qpstat);                   
                    simresult.timing = struct('total',plotvec.time./1e3);
                    simresult.opts.result = 1;
                    if(J.mpcopts.Single)
                        simresult.mode = [upper(mode) ' [Single Precision]'];
                    else
                        simresult.mode = [upper(mode) ' [Double Precision]'];
                    end
                case 'pil'
                    simresult = simPILJMPC(J,simopts);
                otherwise
                    error('Unknown mode selected - options are Matlab, Simulink, MPCToolbox or MEX');                                    
            end            
            simresult = removebias(simresult,J.lin,J.index,J.mpcopts);
        end
        
        %-- Plot Results --%
        function h = plot(J,simresult,varargin)
            %PLOT  Plot Simulation Results 
            %
            %   plot(jMPCobj,jSIMobj) will create a standard plot showing
            %   the result of the simulation.
            %
            %   plot(jMPCobj,jSIMobj,mode) specifies one of three types of
            %   plots available:
            %
            %       'summary'   - standard plot (Default)
            %       'detail'    - multiple figures for simulation detail
            %       'timing'    - timing analysis of simulation
            %
            %   plot(jMPCobj,jSIMobj,mode,plot_model) specifies if you
            %   would like to plot model states and outputs on top of the
            %   measured plant states and outputs. 1 - enabled, 0 -
            %   disabled.
            %
            if(~isempty(varargin)); mode = varargin{1}; else mode = 'summary';  end
            if(length(varargin) > 1); plot_model = varargin{2}; else plot_model = 0; end
            if(~simresult.opts.result)
                error('The supplied jSIM object does not contain simulation results');
            end
            h = [];
            switch(mode)
                case 'summary'
                    h = plotJMPC(J,simresult,mode,plot_model);
                case 'detail'
                    h = plotJMPC(J,simresult,mode,plot_model);
                case 'timing'
                    if(~isempty(simresult.timing))
                        h = timingplot(J,simresult,0);
                    else
                        error('You cannot perform a timing analysis on a Simulink or MATLAB MPC Toolbox Simulation');
                    end
                case 'timing2'
                    if(~isempty(simresult.timing))
                        h = timingplot(J,simresult,1);
                    else
                        error('You cannot perform a timing analysis on a MATLAB MPC Toolbox Simulation');
                    end
                case 'paper'
                    plotMPCPaper(J,simresult);
                otherwise
                error('Valid plot options are summary, detail or timing')
            end
        end
        
        %-- Compare Two Simulations Results --%
        function varargout = compare(J1,simresult1,simresult2,mode)
            %COMPARE  Compare Simulation results from Two MPC Simulations
            %
            %   compare(jMPCobj1,jSIMobj1,jSIMobj2) will create a
            %   standard plot showing a comparison plot of both simulations
            %
            
            if(size(simresult1.plotvec.yp,1) ~= size(simresult2.plotvec.yp,1))
                error('Simulations contain different numbers of outputs!');
            end
            if(size(simresult1.plotvec.u,1) ~= size(simresult2.plotvec.u,1))
                error('Simulations contain different numbers of inputs!');
            end
            if(nargin < 4), mode = 1; end
            if(nargout > 0)
                [varargout{1}, varargout{2}, varargout{3}]= J1.compareplot(simresult1,simresult2,mode);
            else
               J1.compareplot(simresult1,simresult2,mode);
            end                        
        end
        
        %-- Auto Generate Embeddable Code --%
        function [bytes,qpres,mpcres,h] = embed(J,simopts,opts)            
            if(nargin < 3), opts = jMPCeset(); else opts = jMPCeset(opts); end
            if(nargin < 2), simopts = []; end
            %Generate code
            [bytes,qpres,mpcres,h] = embedJMPC(J,simopts,opts);
        end

        %-- Return MATLAB MPC Toolbox Object --%
        function [nMPC] = GetMPCToolboxObj(J,simopts)
            nMPC = buildMatlabMPC(J,simopts);
        end
    end
    
    %Methods in Files not using jMPC object directly
    methods (Static)        
        opts = argcheckMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);  %Check User Inputs
        pred = prediction(Model,QPModel,Np,Nc,Nb,index);    %Create Prediction Matrices
        con = SetupConstraints(con,Phi,Np,Nb,nm_in,n_out,opts);       %Create Inequality Constraints
        [QP,state_est] = create_QP(Model,Phi,Np,Nb,uwt,ywt,con,Kest,sizes,opts); %Create QP Matrices
        MPCobj = buildJMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);  	%Build Model Predictive Controller    
        [optdel_u,u] = saturate_inputs(optdel_u,u,n_in,cons);   %Saturate Inputs
        [dy,du,h] = compareplot(sim1,sim2,plotmode,ispaper);          %Compare two MPC simulations
    end
end


