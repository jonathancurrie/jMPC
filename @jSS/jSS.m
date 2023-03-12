classdef jSS
%jSS  Create a State Space Object
%
%   jSSobj = jSS(lti_obj) creates a jSS object from a MATLAB Control
%   Systems Toolbox Linear Time Invariant (LTI) Object. This can be a State
%   Space (ss), Transfer Function (tf) or Zero-Pole-Gain (zpk) object.
%
%   jSSobj = jSS(A,B,C,D) creates a jSS object from the state space
%   matrices A,B,C,D. This form does not require the Control Systems
%   Toolbox
%
%   jSSobj = jSS(A,B,C,D,Ts) specifies the model sampling period. When
%   omitted, this is assumed to be zero (continuous).
%
%   jSSobj = jSS(A,B,C,D,Ts,x0) specifies the initial states of the model.
%   When omitted, these are assumed to all be zero.
%
%   jSSobj = jSS(...,x0,u_op,x_op) specifies the opearting points of the
%   model i.e. the points it has been linearized about. This flags the
%   Model as been linearized, and therefore now operates in deviation states.
%
%   See also jSS.sim jSS.augment jSS.ssdata jSS.c2d jSS.d2c jSS.d2d
%   jSS.step jSS.impulse

%   Jonathan Currie (C)
%   Control Engineering 2009
    
    properties (SetAccess = private)
        A               % SS A (States) Matrix
        B               % SS B (All Inputs) Matrix
        Bm              % SS B (Measured Input Disturbance) Matrix
        C               % SS C (All Outputs) Matrix 
        Cm              % SS C (Measured Outputs) Matrix 
        D               % SS D (Feed Forward) Matrix
        Ts              % Model Sampling Time
        Map             % Delay2z Mapping Matrix
        linearized      % Flag to indicate built from linearization
        u_op            % Input Operating Point if linearized
        x_op            % State Operating Point if linearized
        y_op            % Output Operating Point if linearized
        meas_out        % Index of measured outputs
        meas_dist       % Index of measured disturbances
        sizes           % Structure of Model Sizes
        isSingle        % Logical whether this model is single precision
    end
    
    properties (SetAccess = public, GetAccess = public)
        x0              % Initial States
    end
    
    properties (SetAccess = private, GetAccess = private)
        has_deadtime    % Internal flag to register dead time present
        InputDelay      % LTI ss property
        OutputDelay     % LTI ss property
        iodelay         % LTI ss property
    end
    
    methods
        %-- Constructor --%
        function Model = jSS(varargin)
            % One Input Argument is assumed to be a Control Systems Toolbox LTI Object
            if(nargin < 2)
                if(isempty(varargin))
                    error('You must supply at least one argument to jSS!');
                end
                check_ctrl();
                if(isa(varargin{1},'ss')) %state space lti
                    system = varargin{1};
                elseif(isa(varargin{1},'tf') || isa(varargin{1},'zpk')) %transfer function or zpk lti
                    system = ss(varargin{1});
                else
                    error('Unknown input form for 1 input argument - only accepts LTI ss, tf or zpk')
                end
                % Check for dead time
                if(hasdelay(system))  
                    if(system.Ts > 0) % System is discrete
                        [system,G] = delay2z(system); %Incorporate dead time into states
                        Model.Map = G;
                        Model.has_deadtime = 0;
                    else
                        Model.has_deadtime = 1; %leave until c2d is called (we don't know sampling time yet)
                        Model.InputDelay = system.InputDelay;
                        Model.OutputDelay = system.OutputDelay;
                        Model.iodelay = system.iodelay;
                        Model.Map = eye(size(system.B,1));
                    end
                else
                    Model.Map = eye(size(system.B,1));
                end
                % Check for linearization points (auto filled in by
                % "linearize"
                if(~isempty(system.UserData))
                    if(isfield(system.UserData,'u_op') && isfield(system.UserData,'x_op'))
                        Model.linearized = 1;
                        Model.u_op = system.UserData.u_op;
                        Model.x_op = Model.Map*system.UserData.x_op;
                        if(isfield(system.UserData,'y_op'))                             
                            Model.y_op = system.UserData.y_op;
                        else
                            Model.y_op = system.C*Model.x_op;
                        end
                    else
                        Model.linearized = 0;
                        Model.u_op = zeros(size(system.B,2),1);
                        Model.x_op = zeros(size(system.B,1),1);
                        Model.y_op = zeros(size(system.C,1),1);
                    end
                else
                    Model.linearized = 0;
                    Model.u_op = zeros(size(system.B,2),1);
                    Model.x_op = zeros(size(system.B,1),1);
                    Model.y_op = zeros(size(system.C,1),1);                    
                end
                % Allocate Class properties
                Model.A = system.A;
                Model.B = system.B;
                Model.C = system.C;                
                Model.D = system.D;
                Model.Ts = system.Ts;
                Model.x0 = zeros(size(Model.B,1),1); %assume 0 initial states
            
            % Multiple arguments are assumed to be SS matrices
            else 
                if(nargin < 4)
                    error('You must supply Model SS Matrices A,B,C,D');                    
                end
                Model.A = varargin{1};
                Model.B = varargin{2};
                Model.C = varargin{3};
                Model.D = varargin{4};
                %Optional Arguments
                if(nargin > 4); Model.Ts = varargin{5}; else Model.Ts = 0; end
                if((nargin > 5) && ~isempty(varargin{6})); Model.x0 = varargin{6}; else Model.x0 = zeros(size(Model.B,1),1); end
                if(nargin > 6) %linearized model supplied
                    if(nargin < 8); error('You must supply both the Input & State Operating Points'); end
                    %Check Sizes
                    if(length(varargin{7}) ~= size(Model.B,2)); error('Incorrect u_op size'); end;
                    if(length(varargin{8}) ~= size(Model.B,1)); error('Incorrect x_op size'); end;
                    
                    Model.u_op = varargin{7};
                    Model.x_op = varargin{8};
                    Model.y_op = Model.C*Model.x_op;
                    Model.linearized = 1;
                else %fill defaults
                    Model.u_op = zeros(size(Model.B,2),1);
                    Model.x_op = zeros(size(Model.B,1),1);
                    Model.y_op = zeros(size(Model.C,1),1);
                    Model.linearized = 0;
                end
                %Check sizes
                if((size(Model.A,1) ~= size(Model.B,1)) || (size(Model.A,1) ~= size(Model.C,2)))
                    error('State Space sizes dont Correlate')
                end
                Model.has_deadtime = 0;
                Model.Map = eye(size(Model.B,1));
            end  
            
            Model.meas_out = 1:size(Model.C,1); %default all outputs are measured
            Model.meas_dist = []; %default all inputs
            Model.Cm = Model.C;
            Model.Bm = [];
            
            %Fill in sizes
            [states,n_in] = size(Model.B);
            n_dist = length(Model.meas_dist);
            n_out = size(Model.C,1);
            nm_out = length(Model.meas_out);            
            Model.sizes = struct('states',states,'n_out',n_out,'n_in',n_in,'nm_out',nm_out,'nm_dist',n_dist,'nm_in',n_in);
            
            %Check for D values
            if(any(Model.D > 0))
                error('The jMPC Toolbox does not currently support direct feedthrough via a state space D matrix');
            end
            
            %Set as Double (default)
            Model.isSingle = false;
        end
        
        %-- Manually Set Linearization Points --%
        function Model = SetLinearization(Model,u_op,x_op)
            if(nargin < 2)
            	error('You must supply u_op and x_op!');                    
            end
            if(size(u_op,2) > size(u_op,1))
                u_op = u_op';
            end
            if(size(x_op,2) > size(x_op,1))
                x_op = x_op';
            end
            Model.u_op = u_op;
            Model.x_op = x_op;
            Model.y_op = Model.C*x_op;
            Model.linearized = 1;
        end
        
        %--Set Unmeasured Outputs --%
        function Model = SetUnmeasuredOut(Model,index)
            n_out = size(Model.C,1);
            if(any(index > n_out))
                error('The maximum output index is %d',n_out)
            elseif(any(index < 1))
                error('The minimum output index is 1');
            end
            %Create a vector of measured outputs (easier to work with)
            v = 1:n_out; v(index) = [];
            index = v;
            
            %Save Measured Outputs
            Model.Cm = Model.C(index,:);
            %Save Vector
            Model.meas_out = index;
            %Update nm_out
            Model.sizes.nm_out = Model.sizes.n_out - length(index);            
        end
        
        %--Set Measured Disturbances --%
        function Model = SetMeasuredDist(Model,index)
            n_in = size(Model.B,2);
            if(any(index > n_in))
                error('The maximum input index is %d',n_in)
            elseif(any(index < 1))
                error('The minimum input index is 1');
            end
            
            %Save Measured Disturbances
            Model.Bm = Model.B(:,index);
            %Save Vector
            Model.meas_dist = index;
            %Update nm_dist & nm_in
            Model.sizes.nm_dist = length(index); 
            Model.sizes.nm_in = n_in - length(index);
        end               
        
        %-- Extract SS Data --%
        function varargout = ssdata(Model)
            %SSDATA  Extract data from a jSS object
            %
            %   [A,B,C,D,Ts,x0] = ssdata(jSSobj) returns all properties of
            %   the model to the user.
            %
            %   ssdata(jSSobj) displays the model properties in the
            %   command window.
            %
            if(nargout == 0) %Simply Display
                format compact
                disp(' ');
                A = Model.A %#ok
                B = Model.B %#ok
                C = Model.C %#ok
                D = Model.D %#ok
                Ts = Model.Ts %#ok
                x0 = Model.x0 %#ok
                disp(' ');
                format loose
            else %Assign to variables
                varargout{1} = Model.A;
            end
            if(nargout > 1); varargout{2} = Model.B; end;
            if(nargout > 2); varargout{3} = Model.C; end;
            if(nargout > 3); varargout{4} = Model.D; end;
            if(nargout > 4); varargout{5} = Model.Ts; end;
            if(nargout > 5); varargout{6} = Model.x0; end;
        end
        
        %-- Convert to Matlab LTI Model
        function ltiModel = ss(Model)
            % Convert to MATLAB State Space LTI Model   
            check_ctrl();
            ltiModel = ss(Model.A,Model.B,Model.C,Model.D,Model.Ts);                
        end
        
        %-- Convert to Discrete -- %
        function dModel = c2d(Model,Ts) 
            % Convert to discrete using Control Systems Toolbox
            % See also c2d            
            check_ctrl();
            if(Ts == 0)
                error('c2d is only used to convert to discrete, use d2c to convert to continuous');
            elseif(Model.Ts > 0)
                error('c2d can only be used on continuous models. use d2d to resample the model.');
            else
                system = ss(Model);                
                %Build dead time into Model
                if(Model.has_deadtime)
                    system.InputDelay = Model.InputDelay; %collect dead time terms
                    system.OutputDelay = Model.OutputDelay;
                    system.iodelay = Model.iodelay;
                    ltiModel = c2d(system,Ts);
                    [ltiModel,G] = delay2z(ltiModel);
                    if(size(G,1) ~= length(Model.x0))
                        warning('jMPC:x0',['Using c2d has changed the number of states, thus x0 is no longer valid. '...
                                           'Either run c2d on a Matlab LTI object, THEN convert to jSS, or ignore this as all states will default to zeros().']);
                        tx0 = [Model.x0; zeros(size(G,1) - length(Model.x0),1)];
                        Model.x0 = G*tx0;
                    else
                        Model.x0 = G*Model.x0;                    
                    end
                    dMap = G;
                else
                    ltiModel = c2d(system,Ts);  
                    dMap = eye(size(ltiModel.B,1));
                end
                %Check for linearized model
                if(Model.linearized)
                    uop = Model.u_op;
                    xop = dMap*Model.x_op;
                    dModel = jSS(ltiModel.A,ltiModel.B,ltiModel.C,ltiModel.D,ltiModel.Ts,Model.x0,uop,xop);                    
                else
                    dModel = jSS(ltiModel.A,ltiModel.B,ltiModel.C,ltiModel.D,ltiModel.Ts,Model.x0); 
                end
                dModel.Map = dMap;
                dModel.Cm = dModel.C(Model.meas_out,:);
                dModel.Bm = dModel.B(Model.meas_dist,:);
            end            
        end
        
        %-- Convert to Discrete with Computational Delay --%
        function dModel = c2dd(Model,Ts,tau)
            % Convert to discrete using Control Systems Toolbox with
            % Computational dead time added
            % See also c2d
            
            % Based on c2dd by J.M.Maciejowski, 25.9.98.
            % Reference: `Predictive control with constraints', section
            %            on `Computational delays'
            check_ctrl();
            if(nargin < 3)
                error('You must supply a Model, Ts and tau');
            end
            if(Ts == 0)
                error('c2d is only used to convert to discrete, use d2c to convert to continuous');
            elseif(Model.Ts > 0)
                error('c2d can only be used on continuous models. use d2d to resample the model.');
            elseif(Model.has_deadtime)
                error('c2dd can only be used on models with no dead time');
            elseif Ts < tau,
                error('tau greater than Ts')
            else
                %Convert Using Maciejowski's Method
                [states,n_in] = size(Model.B);
                n_out = size(Model.C,1);
                [Ad,dummy] = c2d(Model.A,Model.B,Ts);   %#ok<NASGU> % (1,1) block of Ad computed
                [dummy,B1] = c2d(Model.A,Model.B,tau);  %#ok<ASGLU>
                [A2,B2] = c2d(Model.A,Model.B,Ts-tau);  % B2 computed
                B1 = A2*B1;                             % B1 computed
                Ad = [Ad, B1; zeros(n_in,states+n_in)];
                Bd = [B2; eye(n_in)];
                Cd = [Model.C,zeros(n_out,n_in)];                
                dMap = eye(size(Bd,1));
                                                
                %Check for linearized model
                if(Model.linearized)
                    Model.x0 = [Model.x0; Model.u_op];
                    uop = Model.u_op;
                    xop = dMap*Model.x0;
                    dModel = jSS(Ad,Bd,Cd,zeros(n_in,n_out),Ts,Model.x0,uop,xop);                    
                else
                    Model.x0 = [Model.x0; zeros(n_in,1)];  %Add x0
                    dModel = jSS(Ad,Bd,Cd,zeros(n_in,n_out),Ts,Model.x0); 
                end
                dModel.Map = dMap;
                dModel.Cm = dModel.C(Model.meas_out,:);
                dModel.Bm = dModel.B(Model.meas_dist,:);
            end
        end
        
        %-- Resample Discrete -- %
        function dModel = d2d(Model,Ts,varargin)
            % Resample a discrete model using Control Systems Toolbox
            % See also d2d 
            check_ctrl();
            if(~isempty(varargin))
                ltiModel = d2d(ss(Model),Ts,varargin{1}); %allows use of other methods
            else
                ltiModel = d2d(ss(Model),Ts);
            end
            dModel = jSS(ltiModel.A,ltiModel.B,ltiModel.C,ltiModel.D,ltiModel.Ts,Model.x0);
            dModel.Cm = dModel.C(Model.meas_out,:);
            dModel.Bm = dModel.B(Model.meas_dist,:);
        end
        
        %-- Convert to Continuous -- %
        function cModel = d2c(Model)
            % Convert to continuous using Control Systems Toolbox
            % See also d2c 
            check_ctrl();
            if(Model.Ts == 0)
                error('Model is already Continuous!');
            else
                ltiModel = d2c(ss(Model));
                cModel = jSS(ltiModel.A,ltiModel.B,ltiModel.C,ltiModel.D,ltiModel.Ts,Model.x0);
                cModel.Cm = cModel.C(Model.meas_out,:);
                cModel.Bm = cModel.B(Model.meas_dist,:);
            end
        end
        
        %-- Step Response --%
        function varargout = step(varargin)
            % Step Response using Control Systems Toolbox
            % See also step
            % Warning there is a problem when the system has deadtime
            check_ctrl();
            ltiModel = cell(nargin,1);
            for i = 1:nargin
                ltiModel{i} = ss(varargin{i});
            end
            if(nargout > 0)
                [y,t] = step(ltiModel{:});
                varargout{1} = y;
                varargout{2} = t;
            else
                step(ltiModel{:});
            end
        end
        
        %-- Impulse Response --%
        function varargout = impulse(varargin)
            % Impulse Response using Control Systems Toolbox
            % See also impulse
            % Warning there is a problem when the system has deadtime
            check_ctrl();
            ltiModel = cell(nargin,1);
            for i = 1:nargin
                ltiModel{i} = ss(varargin{i});
            end
            if(nargout > 0)
                [y,t] = impulse(ltiModel{:});
                varargout{1} = y;
                varargout{2} = t;
            else
                impulse(ltiModel{:});
            end
        end
        
        %-- Frequency Response --%
        function varargout = bode(varargin)
            % Frequency Response using Control Systems Toolbox
            % See also bode
            % Warning there is a problem when the system has deadtime
            check_ctrl();
            ltiModel = cell(nargin,1);
            for i = 1:nargin
                ltiModel{i} = ss(varargin{i});
            end
            if(nargout > 0)
                [y,t] = bode(ltiModel{:});
                varargout{1} = y;
                varargout{2} = t;
            else
                bode(ltiModel{:});
            end
        end
        
        %-- Discrete Time Linear Quadratic Estimator --%
        function Kest = dlqe(Model,varargin)
            %DLQE  Overloaded version of dlqe
            %
            %   K = dlqe(Model) uses default G, Q & R matrices (Identity)
            %   to calculate the discrete time Kalman Filter
            %
            %   K = dlqe(Model,Q,R) uses the user specified covariance
            %   matrices to calculate the discrete Kalman Filter
            
            check_ctrl();            
            nm_out = length(Model.meas_out);            
            [n_out,states] = size(Model.C);
            
            if(nargin < 2)
                G = eye(size(Model.A));
                Q = eye(size(Model.A));
                if(nm_out ~= n_out)
                    R = eye(size(Model.Cm,1));
                else
                    R = eye(size(Model.C,1));
                end
            elseif(nargin ~= 3)
                error('You Must Supply Q AND R to this function');
            else
                G = eye(size(Model.A));
                Q = varargin{1}; if(size(Q,1) ~= states); error('dlqe Q matrix is the wrong size, expected %1d x %1d',states,states); end
                R = varargin{2}; if(size(R,1) ~= nm_out); error('dlqe R matrix is the wrong size, expected %1d x %1d',nm_out,nm_out); end
            end
            if(Model.Ts <= 0)
                error('The Model Must Be Discrete');
            end
            %Calculate Estimator Gain 
            if(nm_out ~= n_out)               
                Kest = dlqe(Model.A,G,Model.Cm,Q,R); %use only measured outputs
            else
                Kest = dlqe(Model.A,G,Model.C,Q,R);
            end
        end
        
        %-- Augment Outputs to State -- %
        function aModel = augment(Model)
            %AUGMENT  Augment the model's output to the state matrix 
            %
            %   aug_jSSobj = augment(jSSobj) returns the augmented form of
            %   the supplied jSS model
            
            % Modified from Liuping Wang's original function listed in her
            % book, "Model Predictive Control System Design &
            % Implementation Using MATLAB"
            
            %Collect sizes
            [states,n_in] = size(Model.B);
            n_out = size(Model.C,1);

            %Augment Matrix
            Ae = eye(states+n_out,states+n_out);
            Ae(1:states,1:states) = Model.A;
            Ae(states+1:states+n_out,1:states) = Model.C*Model.A;

            Be = zeros(states+n_out,n_in);
            Be(1:states,:) = Model.B;
            Be(states+1:states+n_out,:) = Model.C*Model.B;

            Ce = zeros(n_out,states+n_out);
            Ce(:,states+1:states+n_out) = eye(n_out);

            De = Model.D;

            %Add Initial Model Output to Model States
            ym0 = Model.C*Model.x0;
            %Solve State Increment
            if(rank(Ae(1:states,1:states)) >= states)
                xm0 = Ae(1:states,1:states)\(Model.A*Model.x0-Model.x0); %try get initial delta x 
            else
                xm0 = zeros(states,1);
            end
            %Create new SS Model
            aModel = jSS(Ae,Be,Ce,De,Model.Ts,[xm0; ym0]);
        end
        
        %-- Simulate Model For a Given Input --%
        function [x,y] = sim(Model,x,u,varargin)
            %SIM  Simulate a jSS Linear State Space Model 
            %
            %   x[k+1] = Model.A*x[k] + Model.B*u[k]
            %   y[k+1] = Model.C*x[k+1]
            %
            %   [x,y] = sim(jSSobj,x,u) simulates the linear model over one
            %   time step, returning the model's states and output.
                        
            x = Model.A*x + Model.B*u;    
            y = Model.C*x;
        end
        
        %-- Convert to Single Precision --%
        function S = single(S)
           %SINGLE Convert the jSS Object to Single Precision
            S.A = single(S.A);
            S.B = single(S.B);
            S.Bm = single(S.Bm);
            S.C = single(S.C);
            S.Cm = single(S.Cm);
            S.D = single(S.D);
            S.Ts = single(S.Ts);
            S.Map = single(S.Map);
            S.linearized = single(S.linearized);
            S.u_op = single(S.u_op);
            S.x_op = single(S.x_op);
            S.y_op = single(S.y_op);
            S.meas_out = single(S.meas_out);
            S.meas_dist = single(S.meas_dist);
            S.x0 = single(S.x0);
            sz = S.sizes;
            S.sizes = struct('states',single(sz.states),'n_out',single(sz.n_out),...
                             'n_in',single(sz.n_in),'nm_out',single(sz.nm_out),...
                             'nm_dist',single(sz.nm_dist),'nm_in',single(sz.nm_in));
            S.isSingle = true;
        end
        
        %-- Convert to Double Precision --%
        function S = double(S) %#ok<MANU>
           error('Converting from single to double will lose initial model precision, and thus is disabled');
        end
    end
    
end

function check_ctrl()
%Check Control Systems Toolbox is installed
    c = ver('control');
    if(isempty(c))
        error('The MATLAB Control Systems Toolbox must be installed');
    end
end