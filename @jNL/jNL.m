classdef jNL < handle
%jNL  Create a Non Linear Model Object
%
%   jNLobj = jNL(xfnc,C) creates the jNL object with the nonlinear function
%   specified as a function handle, xfnc. C is either a matrix (n_out x 
%   states) or a function handle, specifying the relationship between the 
%   internal states and the model output.
%   
%   jNLobj = jNL(xfnc,C,param) specifies additional parameters to be passed
%   to the nonlinear function fnc. These must be a cell array such as:
%   
%       param = {l,m,n};
%
%   jNLobj = jNL(xfnc,C,param,x0) specifies the initial states of the nonlinear 
%   model. When this argument is omitted, the initial states are assumed to
%   be all zero.
%
%   Note: The current implementation only accepts continuous nonlinear
%   functions (ODEs) - which are written in standard Cauchy (1st Order)
%   form. A MATLAB ODE solver is used as the numerical integrator at each time 
%   step. The ODE must be time independent (i.e. xdot = f(x,u)). An example
%   is:
%
%       function xdot = nl_fcn(t,x,u,param)
%           xdot(1,1) = x(2);
%           xdot(2,1) = param{1}*sin(u);
%
%   See also jNL.sim jSS

%   Jonathan Currie (C)
%   Control Engineering 2011
    
    properties        
        param       % model parameters (optional)
        lin_model   % optional field to store the linearized model in
        x0          % initial state (optional assumed 0)
    end
    
    properties (SetAccess = private)
        nl_model    % function handle to non-linear model
        C           % relates outputs to states        
        odesolver   % ode solver used for simulation
        isSingle    % Logical whether this model is single precision
    end
    
    properties (GetAccess = private, SetAccess = private)
        newx0       % is x0 a guess
    end
    
    %Class Only Methods
    methods
        %-- Constructor --%
        function J = jNL(fnc,C,varargin)
            if(nargin < 2)
                error('You must supply a function handle & C matrix / C fcn');
            end
            if(~isempty(varargin))
                eparam = varargin{1};
                if(~isa(eparam,'cell'))
                    error('ODE Parameters must be passed as a cell array');
                end
                J.param = eparam;
                if(length(varargin) > 1)
                    if(isempty(varargin{2}))
                        if(isa(C,'function_handle'))
                            error('You must supply an initial state vector when C is a function handle');
                        end                        
                        J.x0 = zeros(size(C,2),1); %assume initial states are zero
                        J.newx0 = true;
                    else
                        J.x0 = varargin{2};
                        J.newx0 = false; %user supplied initial states
                        [r2,c2] = size(J.x0);
                        if(c2 > r2)
                           J.x0 = J.x0';
                        end
                    end
                else
                    if(isa(C,'function_handle'))
                        error('You must supply an initial state vector when C is a function handle');
                    end                        
                    J.x0 = zeros(size(C,2),1); %assume initial states are zero
                    J.newx0 = true;
                end
            else
                if(isa(C,'function_handle'))
                    error('You must supply an initial state vector when C is a function handle');
                end   
                J.x0 = zeros(size(C,2),1); %assume initial states are zero
                J.newx0 = true;
                J.param = [];
            end

            %Basic error checking
            if(~isa(fnc,'function_handle'))
                error('You must suppply a function handle to the nonlinear model');
            end

            %Assign Class properties
            J.nl_model = fnc;
            J.lin_model = []; %default empty
            J.C = C;
            J.odesolver = 0; %default ode45
            J.isSingle = false;
        end       
        
        %Set the ODE Solver
        function SetSolver(J,odeS)
            %SetSolver  Manually set the ODE solver
            %
            %   setSolver(jNLobj,odeS) manually sets the solver used for
            %   numerical integration when simulating the jNL object. This
            %   property is typically set when the object is linearized,
            %   however this method is provided to manually override or set
            %   this property. Valid solvers are 'ode45', 'ode23s', or
            %   'ode15s'.
            
            switch(odeS)
                case 'ode45'; J.odesolver = 0;
                case 'ode23s'; J.odesolver = 1;
                case 'ode15s'; J.odesolver = 2;
                otherwise; error('Valid ODE Solvers for jNL are ode45, ode23s and ode15s');
            end                       
        end
        
        function J = SetPlantOutputs(J,C)
            %SetPlantOutputs  Manually update the object C matrix
            %
            %   SetPlantOutputs(jNLobj,C)
            
                J.C = C;
        end
        
        %Simulate Nonlinear Model
        function [x,y] = sim(J,x0,u,Ts)
            %SIM  Simulate a jNL Nonlinear Model
            %
            %   [x,y] = SIM(jNLobj,x0,u,Ts) simulates the nonlinear model
            %   specified in jNL, using x0 as the initial states, and u as
            %   the current model input. Integration is performed between 0
            %   and Ts using a preset ODE solver.
            %
            
            %Ensure All Doubles
            x0 = double(x0); u = double(u); Ts = double(Ts);            
            switch(J.odesolver)
                case(0)
                    [t,x] = ode45(@(t,x) J.nl_model(t,x,u,J.param),[0 Ts],x0); %#ok<ASGLU>
                case(1)
                    [t,x] = ode23s(@(t,x) J.nl_model(t,x,u,J.param),[0 Ts],x0); %#ok<ASGLU>
                case(2)
                    [t,x] = ode15s(@(t,x) J.nl_model(t,x,u,J.param),[0 Ts],x0); %#ok<ASGLU>
            end
            x = x(end,:)';
            if(isa(J.C,'function_handle')) %non linear output function
                y = J.C([],x,u,J.param);
            else  %linear output              
                y = J.C*x;
            end
            if(J.isSingle)
                x = single(x);
                y = single(y);
            end
        end
        
        function [Model,x_op] = linearize(J,u_op,x_op,odeS,verb)
            %LINEARIZE  Linearize a collection of ODEs about an operating 
            %           point
            %   
            %   [ltiModel,x_op] = linearize(jNLobj,u_op) linearizes the jNL
            %   nonlinear model about input operating point, u_op, and a
            %   solved steady state x_op. The nonlinear model MUST be
            %   stable for this routine to work.
            %
            %   [ltiModel] = linearize(jNLobj,u_op,x_op) linearizes the jNL
            %   nonlinear model about the supplied input and state
            %   operating points. This is the preferred method of calling
            %   this function.
            %
            %   [litModel] = linearize(jNLobj,u_op,[],odeSolver)
            %   linearizes the jNL model using the selected ODE solver.
            %   Valid options are 'ode45', 'ode23s', and 'ode15s'. Note
            %   the operating state (x_op) need not be supplied.
            %
            %   This function will attempt to solve for the steady using
            %   the selected ode solver. If it fails, it will try using
            %   fsolve. If it still fails, it will try a selection of
            %   relaxation methods in order to find a solution.
            %
            %   Important to note is this function can and will fail. When
            %   you can analytically solve your nonlinear function
            %   (e.g. using the Symbolic Toolbox), it is highly recommended
            %   you do this!
            %
            %   Simulink also provides two suitable & robust methods if you
            %   have a Simulink Model of your nonlinear model (which is
            %   easy to build). Try "trim" for obtaining a steady state point
            %   and "linmod2" to linearize about this point.
            
            if(nargin < 5), verb = 0; end
            if(nargin < 4 || isempty(odeS)), odeS = 'ode45'; end
            if(nargin < 3), x_op = []; end
            if(nargin < 2), error('This function requires at least 2 inputs'); end
            
            if(size(u_op,2) > size(u_op,1)); u_op = u_op'; end
            if(size(x_op,2) > size(x_op,1)); x_op = x_op'; end
            %Ensure All Doubles
            u_op = double(u_op); x_op = double(x_op);
            
            %Select Solver
            switch(odeS)
                case 'ode45'; J.odesolver = 0;
                case 'ode23s'; J.odesolver = 1;
                case 'ode15s'; J.odesolver = 2;
                otherwise; error('Valid ODE Solvers for jNL are ode45, ode23s and ode15s');
            end
            
            if(verb)
                fprintf('\n------------------------------------------------\n');
                fprintf('Linearizing Model: %s\n\n',func2str(J.nl_model));
            end
            if(isempty(x_op))
                %Check if x0 is a guess (can get a starting go from selected solver)
                if(J.newx0)
                    switch(J.odesolver)
                        case(0)
                            [t,x] = ode45(@(t,x) J.nl_model(t,x,u_op,J.param),[0 10],J.x0); %#ok<ASGLU>
                        case(1)
                            [t,x] = ode23s(@(t,x) J.nl_model(t,x,u_op,J.param),[0 10],J.x0); %#ok<ASGLU>
                        case(2)
                            [t,x] = ode15s(@(t,x) J.nl_model(t,x,u_op,J.param),[0 10],J.x0); %#ok<ASGLU>
                    end
                    x0 = x(end,:);
                else
                    x0 = J.x0;
                end
                %Solve Steady State Opearting Point
                [x_op,flag] = J.jsolve_ss(J.nl_model,u_op,x0,J.param,J.odesolver,verb);
                if(flag < 0)
                    error(['Could not find Steady State Operating Point. The system is '... 
                        'either unstable, or you may need to choose another point '...
                        'to linearize about. If you already know the point you wish '...
                        'to linearize about, call "linearize(fcn,u_op,x_op)", or '...
                        'alternatively if you have a simulink model, try using "trim"']);
                end
            else
                [r,c] = size(x_op);
                if(c > r); x_op = x_op'; end
            end

            %Try use Intel's djacobi if available
            haveDJAC = false;
            if(exist('djacobi','file') == 3)
                haveDJAC = true;     
                A = djacobi(@(x) J.nl_model(1,x,u_op,J.param), x_op);
                B = djacobi(@(u) J.nl_model(1,x_op,u,J.param), u_op);
            else    
                f = @(t,x) J.nl_model(t,x,u_op,J.param);
                f2 = @(t,u) J.nl_model(t,x_op,u,J.param);    
                A = numjac(f,1,x_op,f(1,x_op),1e-6,[]);
                B = numjac(f2,1,u_op,f2(1,u_op),1e-1,[]);
            end
            
            if(isa(J.C,'function_handle'))
                if(haveDJAC)
                    C = djacobi(@(x) J.C(1,x,u_op,J.param), x_op);
                    D = djacobi(@(u) J.C(1,x_op,u,J.param), u_op);
                else
                    g = @(t,x) J.C(t,x,u_op,J.param);
                    g2 = @(t,u) J.C(t,x_op,u,J.param);
                    C = numjac(g,1,x_op,g(1,x_op),1e-6,[]); 
                    D = numjac(g2,1,u_op,g2(1,u_op),1e-6,[]);
                end                
            else %already linear output function
                C = J.C; %#ok<*PROP>
                D = zeros(size(C,1),length(u_op)); %jMPC does not use D matrix
            end
            
            if(verb)
                fprintf('------------------------------------------------\n\n');
            end
            
            %Return Matlab LTI Object (so we can still add iodelay later..)        
            Model = ss(A,B,C,D);
            %Fill In User Data with Steady State Information
            Model.UserData.u_op = u_op;
            Model.UserData.x_op = x_op;
            Model.UserData.y_op = Model.C*x_op;
        end
        
        
        %Simulate Nonlinear Model and return all values
        function [t,y] = odesim(J,x0,u,T,varargin)
            %ODESIM  Simulate a jNL Nonlinear Model and return all values
            %
            %   [t,y] = SIM(jNLobj,x0,u,T) simulates the nonlinear model
            %   specified in jNL, using x0 as the initial states, and u as
            %   the current model input. Integration is performed between 0
            %   and T using a preset ODE solver.
            %
            if(~isempty(varargin)); odeopts = varargin{1}; else odeopts = odeset(); end            
            switch(J.odesolver)
                case(0)
                    [t,x] = ode45(@(t,x) J.nl_model(t,x,u,J.param),[0 T],x0,odeopts); 
                case(1)
                    [t,x] = ode23s(@(t,x) J.nl_model(t,x,u,J.param),[0 T],x0,odeopts); 
                case(2)
                    [t,x] = ode15s(@(t,x) J.nl_model(t,x,u,J.param),[0 T],x0,odeopts); 
            end
            %Preallocate y
            if(isa(J.C,'function_handle'))
                ytemp = J.C([],x(1,:),u,J.param);
                nout = length(ytemp);
            else
                nout = size(J.C,1);
            end
            y = zeros(length(t),nout);
            %Find y
            for i = 1:length(t)
                if(isa(J.C,'function_handle')) %nonlinear output function
                    y(i,:) = J.C([],x(i,:),u,J.param);
                else  %linear output              
                    y(i,:) = J.C*x(i,:)';
                end
            end
        end
        
        %Set As Single Precision (bit of a hack for now)
        function J = single(J)
            J.isSingle = true;
        end
        
    end
    
    methods (Static)
        %-- Solve for Steady State --%
        [x_op,exitflag] = jsolve_ss(fcn,u0,n,param,odeS,verb);
    end
end