function opts = argcheckMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts)
%Detect Incorrect User Inputs for creating a jMPC object
%
%   Called By jMPC Constructor

%   Jonathan Currie (C)
%   Control Engineering 2011

%Check QP solver (empty means use internal heuristic)
if(isempty(opts.QPSolver)), opts.QPSolver = 'mehrotra'; end
%Check for wright/mehrotra
switch(lower(opts.QPSolver))
    case 'mehrotra'
        if(exist('mquad_mehrotra','file') == 3)
            if(opts.Single)
                opts.QPSolver = 'msquad_mehrotra'; 
            else
                opts.QPSolver = 'mquad_mehrotra';
            end
        else
            if(opts.Single)
                opts.QPSolver = 'squad_mehrotra';
            else
                opts.QPSolver = 'quad_mehrotra';
            end
        end
    case 'wright'
        if(exist('mquad_wright','file') == 3)
            if(opts.Single)
                opts.QPSolver = 'msquad_wright'; 
            else
                opts.QPSolver = 'mquad_wright';
            end
        else
            if(opts.Single)
                opts.QPSolver = 'squad_wright';
            else
                opts.QPSolver = 'quad_wright';
            end
        end
end
    

%Check QP solver tol (empty means use internal heuristic)
if(isempty(opts.QPTol))
    if(opts.Single)
        opts.QPTol = 1e-4; %single precision
    elseif(any(strcmpi(opts.QPSolver,{'squad_mehrotra','msquad_mehrotra','squad_wright','msquad_wright'})))
        opts.QPTol = 1e-4; %single precision 
    else
        opts.QPTol = 1e-6; %double precision
    end
end

%Check interfaced solvers if requested
switch(lower(opts.QPSolver))
    case 'quadprog'
        if(exist('quadprog','file') ~= 2)
            error('jMPC Cannot Locate MATLAB''s quadprog, please select another QP solver');
        end
    case {'qpip','qpas','clp','ooqp'}
        if(exist(lower(opts.QPSolver),'file') ~= 3)
            error('jMPC Cannot Locate %s, please select another QP solver',lower(opts.QPSolver));
        end
end

%Check Options structure
try
    opts = jMPCset(opts);
catch ME
    error(sprintf(['Options Structure Error:\n' ME.message])); %#ok<SPERR>
end

%Setup default quadprog options if required
if(strcmpi(opts.QPSolver,'quadprog') && isempty(opts.QuadProgOpts))
    try
        opts.QuadProgOpts = optimset('Algorithm','interior-point-convex','Display','off');
    catch
        opts.QuadProgOpts = [];
    end
end

%Setup mex warning level
opts.mexWarn = any(strcmpi(opts.Warnings,{'all','critical'}));

%Check sensible solver with single precision run
if(opts.Single && ~any(strcmpi(opts.QPSolver,{'squad_wright','msquad_wright','msquad_wrightLV','squad_mehrotra','msquad_mehrotra','pil'})))
    error('Please select a single precision solver (such as squad_wright) when building and simulating a single precision MPC controller');
end

%Check obvious points
if(isempty(Model))
    error('No Model Supplied')
end
if(~isa(Model,'jSS') && ~isa(Model,'jNL'))
    error('The Model must be a jSS or jNL object')
end

%If Nonlinear Simulation use Initial Linear Model
if(isa(Model,'jNL'))
    error('Nonlinear MPC is currently under development!');   
%     if(isempty(Model.lin_model))
%         error('You must supply an initial linearized model for a Nonlinear Simulation');
%     end
%     Model = Model.lin_model;
end

%Check for linearized points if lin_sim is enabled
if(opts.LinSim && ~Model.linearized)
	error('You have specified a linear simulation with linearization points via opts.lin_sim but the model is not linearized!');
end

%Check Model is Discrete
if(Model.Ts <= 0)
    error('The Model must have a Discrete Sample Time')
end

%Collect sizes
[n_out,states] = size(Model.C);
n_in = size(Model.B,2);
nm_dist = length(Model.meas_dist);
nm_in = n_in - nm_dist;
    
%Check Blocking
[n,m] = size(Nc);
if(n==1 && m==1) %Single Control Horizon
    if(Nc > Np)
        error('Prediction Horizon MUST be >= than Control Horizon')
    end
else
    if(n>1 && m>1)
        error('Blocking Moves must be a Vector!');
    end
    if(n>Np || m>Np)
        error('You cannot have more blocking moves than Np samples!');
    end
    if(any(Nc < 1))
        error('You cannot have a blocking move less than 1!');
    end
end

%Check correct number of weight terms
if(length(uwt) ~= nm_in)
    error('Incorrect Number of Input Weights (uwt) - Expected %1d x %1d',1,nm_in)
end

if(length(ywt) ~= n_out)
    error('Incorrect Number of Output Weights (ywt) - Expected %1d x %1d',1,n_out)
end

%Check for initial condition error
if(length(Model.x0) ~= states)
    error('Incorrect Number of Model Initial Conditions (Model.x0) - Expected %1d x %1d',states,1)
end
if(~isempty(opts.InitialU))
    if(length(opts.InitialU) ~= n_in)
        error('Incorrect length of opts.InitialU - Expected %1d x %1d',n_in,1);
    end
end

%Check correct number of Input Labels
if(~isempty(opts.InputNames))
    if(length(opts.InputNames) ~= n_in)
        error('Incorrect Number of Input Name Labels (InputNames) - Expected %1d x %1d',1,n_in);
    end
end
if(~isempty(opts.InputUnits))
    if(length(opts.InputUnits) ~= n_in)
        error('Incorrect Number of Input Unit Labels (InputUnits) - Expected %1d x %1d',1,n_in);
    end
end
%Check correct number of Output Labels
if(~isempty(opts.OutputNames))
    if(length(opts.OutputNames) ~= n_out)
        error('Incorrect Number of Output Name Labels (OutputNames) - Expected %1d x %1d',1,n_in);
    end
end
if(~isempty(opts.OutputUnits))
    if(length(opts.OutputUnits) ~= n_out)
        error('Incorrect Number of Output Unit Labels (OutputUnits) - Expected %1d x %1d',1,n_in);
    end
end

%Check State Estimator Gain
if(~isempty(Kest))
    [r,c] = size(Kest);
    if(r ~= states)
        error('Incorrect State Estimtation Gain Size (Kest) - Expected %1d rows',states);
    end
    if(c ~= length(Model.meas_out));
            error(['The number of Measured Outputs do not correspond with the State Estimation Gain Size (Kest).'...
                '\nYou have specified %1d measured outputs, while there are %1d columns in Kest.'],length(Model.meas_out),c);
    end
end
       
%Check constraints
if(isfield(con,'u') && ~isempty(con.u))
    [r,c] = size(con.u);
    if((r ~= nm_in) || (c ~= 3))
        error('Incorrect Number of Input Constraints (con.u) - Expected %1d x %1d',nm_in,3)
    end
    for i = 1:r
        if(con.u(i,1) >= con.u(i,2))
            error('Umin(%d) Constraint >= Umax(%d) Constraint',i,i)
        end
        if(con.u(i,3) <= 0)
            error('Delumax(%d) <= 0',i)
        end
    end
end

if(isfield(con,'y') && ~isempty(con.y))
    [r,c] = size(con.y);
    if((r ~= n_out) || (c ~= 2))
        error('Incorrect Number of Output Constraints (con.y) - Expected %1d x %1d',n_out,2)
    end 
    for i = 1:r
        if(con.y(i,1) >= con.y(i,2))
            error('Ymin(%d) Constraint >= Ymax(%d) Constraint',i,i)
        end
    end
end

if(isfield(con,'slack') && ~isempty(con.slack))
   if(any(con.slack <= 0))
       error('Slack weight cannot be zero or less');
   else if(any(con.slack > 1e6))
           %If Inf then we are dropping this slack term
           noinf = con.slack(~isinf(con.slack));
           if(any(noinf > 1e6))
                error('Slack weight cannot be greater than 1e6');
           end
       end
   end
   %Check for mixed hard/soft
%    if(length(con.slack) > 1)
       %Check same number of slack weights as outputs
       if(length(con.slack) ~= n_out)
           error('Incorrect Number of Slack Weights (con.slack) - Expected %1d x 1 (nout x 1)',n_out);
       end
       %Check finite slack weights correspond to finite constraints
       if(isfield(con,'y') && ~isempty(con.y))
            iy = ~isinf(con.y(:,1)) | ~isinf(con.y(:,2));
            for i = 1:length(con.slack)
                if(~isinf(con.slack(i)) && ~iy(i))
                    error('Slack weight on output %d is finite, yet no constraint is specified on this output!',i);
                end
            end
       else
            error('You cannot specify slack weights when you have no output constraints!');
       end
%    end
   
end

end

