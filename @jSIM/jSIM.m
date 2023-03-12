classdef jSIM
%jSIM  Create a Simulation Options Object
%
%   jSIMobj = jSIM(jMPCobj,Plant,T,setp) creates a jSIM object which is
%   used for creating the simulation environment. Data is extracted from 
%   the supplied controller (jMPCobj) to verify supplied objects. Argument
%   Plant can be a jSS object (for linear models) or a jNL object (for
%   non-linear models). T specifies the number of samples for the
%   simulation, while setp is a column vector of the desired setpoints.
%
%   jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist) specifies an unmeasured
%   input disturbances to be used during the simulation. This is a column 
%   matrix. When omitted (or empty), no disturbance is added.
%
%   jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist) specifies a measured
%   input disturbance to be used during the simulation. This is a
%   column matrix. When omitted (or empty), no disturbance is added.
%
%   jSIMobj = jSIM(jMPCobj,Plant,T,setp,umdist,mdist,noise) specifies output
%   (measurement) disturbances to be used during the simulation. This is a
%   column matrix. When omitted (or empty), no disturbance is added.
%
%   See also jMPC jSS jNL

%   Jonathan Currie (C)
%   AUT University 2011
    
    properties
        mode        % Simulation Mode
        Plant       % Simulation Plant
        T           % Number of Samples for Simulation
        Ts          % Controller Sampling Period    
        setp        % Simulation Setpoint
        udist       % Unmeasured Input Disturbances
        mdist       % Measured Input Disturbances
        ydist       % Output (Measurement Noise) Disturbances        
        initial     % Initial Plant Values (xp, yp)  
        plotvec     % Structure containing logged results
        timing      % Structure of execution times at each sample
        qpstats     % Structure of QP solver statistics
        QP          % Saved QP problem
        opts        % Internal Options (Type & Disturbances Present)
    end
    
    properties(SetAccess = private)
        simulink    % Structures for Simulink Model Simulation (From Workspace)
        warning     % Warnings generated during construction        
    end
    
    methods
        %Constructor
        function S = jSIM(MPCobj,Plant,T,setp,varargin)
            %Check Optional Inputs
            if(~isempty(varargin)); umdist = varargin{1}; else umdist = []; end
            if(length(varargin) > 1); measdist = varargin{2}; else measdist = []; end
            if(length(varargin) > 2); outdist = varargin{3}; else outdist = []; end
            %Check User Inputs
            jSIM.argcheckSIM(Plant,MPCobj.sizes,T,setp,umdist,measdist,outdist,MPCobj.mpcopts);
            %Build Simulation Options
            simopts = jSIM.buildSIM(MPCobj,Plant,T,setp,umdist,measdist,outdist);
            
            %Allocate Class Properties
            S.Plant = Plant;
            S.T = T;    
            S.Ts = simopts.Ts;
            S.setp = simopts.setp;
            S.udist = simopts.udist;
            S.ydist = simopts.ydist;
            S.mdist = simopts.mdist;
            S.initial = simopts.initial; 
            S.plotvec = simopts.plotvec;
            S.simulink = simopts.simulink;
            S.opts = simopts.opts;
            S.warning = simopts.warning;
        end
       
        %Remove Bias
        function S = removebias(S,lin,index,opts)
            %REMOVEBIAS  Remove opearting point biases from model results
            %
            %   jSIMobj = removebias(jSIMobj,lin,index,opts) uses the
            %   supplied opearting points to modify the jSIM results object
            %   such that values reflect true states, not deviation states.
            %
            p = S.plotvec; len = length(p.u(:,1));
            u_op = lin.u_op; x_op = lin.x_op; y_op = lin.y_op_full;           
            
            if(isempty(strfind(S.mode,'Simulink'))) %Simulink adds bias due to plant in model
                %if(~opts.lin_sim)
                    p.u = p.u + repmat(u_op',len,1);
                %end
            else %First point in Simulink does not reflect operating states
                p.ym(1,:) = p.ym(1,:) - y_op';
                p.xm(1,:) = p.xm(1,:) - x_op';
            end                
                
            if(~isempty(p.ym))
                p.ym = p.ym+repmat(y_op',len,1);
            end
            p.xm = p.xm+repmat(x_op',len,1);            
            
            if(opts.LinSim)
                p.yp = p.yp+repmat(lin.y_op',len,1);
            end
            
            S.setp = S.setp+repmat(y_op(index.iq_out)',size(S.setp,1),1);
            S.plotvec = p;
        end
        
        %Convert To Single
        function S = single(S)
            %SINGLE  Convert MPC Simulation Data to Single Precision            
            S.Plant = single(S.Plant);
            S.T = single(S.T);
            S.Ts = single(S.Ts);
            S.setp = single(S.setp);
            S.udist = single(S.udist);
            S.mdist = single(S.mdist);
            S.ydist = single(S.ydist);
            S.initial = struct2single(S.initial);
            S.plotvec = struct2single(S.plotvec);
        end
    end
    
    %Methods in Files
    methods (Static)        
        %-- Check User Inputs --%
        argcheckSIM(Plant,sizes,T,Ts,setp,umdist,measdist,outdist,opts);
        %-- Build Simulation Options Structure --%
        simopts = buildSIM(MPCobj,Plant,T,Ts,setp,umdist,measdist,outdist);
    end
        
    
end

