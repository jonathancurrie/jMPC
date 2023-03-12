function options = jMPCset(varargin)
%JMPCSET  Create or alter the options for a jMPC Simulation
%
% Code is heavily based on optimset (C) Mathworks

% Jonathan Curre

% Print out possible values of properties.
if (nargin == 0) && (nargout == 0)
    printfields();
    return
end

Names = {'ScaleSystem';'ScaleFac';'LookAhead';'InitialU';'Single';'DualMode';'QPSolver';'QPMaxIter';'QPTol';'QPWarmStart';'QPVerbose';'QPSave';'QuadProgOpts';'InputNames';'InputUnits';...
             'OutputNames';'OutputUnits';'LinSim';'Warnings';'WaitBar'};
Defaults = {1,1,0,[],0,0,[],30,[],1,0,0,[],[],[],[],[],0,'critical',1};         


%Transpose as required
if(size(Names,2) > 1), Names = Names'; end
if(size(Defaults,2) > 1), Defaults = Defaults'; end

%Collect Sizes and lowercase matches         
m = size(Names,1); 
numberargs = numel(varargin);
%Create structure with all names and default values
st = [Names,Defaults]'; options = struct(st{:});

% Check we have char or structure input. If structure, insert arguments
i = 1;
while i <= numberargs
    arg = varargin{i};
    if ischar(arg)
        break;
    end
    if ~isempty(arg)
        if ~isa(arg,'struct')
            error('An argument was supplied that wasn''t a string or struct!');
        end
        for j = 1:m
            if any(strcmp(fieldnames(arg),Names{j,:}))
                val = arg.(Names{j,:});
            else
                val = [];
            end
            if ~isempty(val)
                checkfield(Names{j,:},val);
                options.(Names{j,:}) = val;
            end
        end
    end
    i = i + 1;
end

%Check we have even number of args left
if rem(numberargs-i+1,2) ~= 0
    error('You do not have a value specified for each field!');
end

%Go through each argument pair and assign to correct field
expectval = 0; %first arg is a name
while i <= numberargs
    arg = varargin{i};
    switch(expectval)
        case 0 %field
            if ~ischar(arg)
                error('Expected fieldname (argument %d) to be a string!',i);
            end
            j = find(strcmp(arg,Names) == 1);
            if isempty(j)  % if no matches
                error('Unrecognised parameter ''%s''',arg);
            elseif(length(j) > 1)
                error('Ambiguous parameter ''%s''',arg);
            end
            expectval = 1; %next arg is a value
        case 1
            if(~isempty(arg))   
                if(ischar(arg)), arg = lower(arg); end
                checkfield(Names{j,:},arg);
                options.(Names{j,:}) = arg;
            end
            expectval = 0;
    end
    i = i + 1;
end

if expectval %fallen off end somehow
    error('Missing value for final argument ''%s''',arg);
end



function checkfield(field,value)
%Check a field contains correct data type
err = [];
switch lower(field)
    %Scalar integer within limits
    case 'qpmaxiter' 
        err = scalar_int_lim(value,field,1,1e5); %1 <= iter <= 1e5
    case 'qpverbose' 
        err = scalar_int_lim(value,field,0,2); %0 <= iter <= 2
    case 'dualmode' 
        err = scalar_int_lim(value,field,0,1); %0 <= iter <= 1
    case 'qpsave'
        err = scalar_int_lim(value,field,0,1e9); %0 <= iter <= 1e9
    case {'lookahead','linsim','qpwarmstart','scalesystem','single','waitbar'}
        err = scalar01(value,field);
    %Scalar double within limits
    case 'qptol'
        err = scalar_real_lim(value,field,1e-15,1); %1e-15 <= tol <= 1
    case 'scalefac'
        err = scalar_real_lim(value,field,1e-2,1e6); %0.01 <= tol <= 1e6
    %Double vector/matrix
    case 'initialu'
        if(~isnumeric(value) || ~isreal(value) || issparse(value) || (size(value,1) > 1 && size(value,2) > 1))
            err = MException('jMPC:SetFieldError','Parameter ''%s'' should be a real double/single dense vector',field);
        end        
    %Valid String
    case 'warnings'
        err = checkString(value,field,{'all','critical','none'});
    case 'qpsolver'
        err = checkString(value,field,{'mehrotra','quad_mehrotra','mquad_mehrotra','squad_mehrotra','msquad_mehrotra',...
                                       'wright','quad_wright','mquad_wright','squad_wright','msquad_wright',...
                                       'quad_hildreth','quadprog','qpip','qpas','qpOASES','ooqp','pil'});    
    %Structure
    case 'quadprogopts'
        if(~isstuct(value)), err = MException('jMPC:SetFieldError','Parameter ''%s'' should be an optimset structure',field);  end
    %Cell Array
    case {'inputnames','inputunits','outputnames','outputunits'}
        if(~iscell(value)), err = MException('jMPC:SetFieldError','Parameter ''%s'' should be a cell array of strings',field);  end
        
    otherwise  
        err = MException('jMPC:SetFieldError','Unrecognized parameter name ''%s''.', field);
end
if(~isempty(err)), throw(err); end


function err = scalar01(value,field)
%Scalar 0 or 1
err = [];
if(~isscalar(value) || ~isnumeric(value) || (value ~= 0 && value ~=1))
    err = MException('jMPC:SetFieldError','Parameter ''%s'' should be an integer scalar with the value 0 or 1',field);
end

function err = scalar_int_lim(value,field,lb,ub)
%Scalar integer lb <= value < ub
err = [];
if(~isscalar(value) || ~isnumeric(value) || ~isreal(value) || (round(value) ~= value) || value < lb || value > ub || (~isa(value,'double') && ~isa(value,'single')))
    err = MException('jMPC:SetFieldError','Parameter ''%s'' should be a scalar integer %g <= value <= %g',field,lb,ub); 
end

function err = scalar_real_lim(value,field,lb,ub)
%Scalar real lb <= value < ub
err = [];
if(~isscalar(value) || ~isnumeric(value) || ~isreal(value) || value < lb || value > ub || (~isa(value,'double') && ~isa(value,'single')))
    err = MException('jMPC:SetFieldError','Parameter ''%s'' should be a scalar real %g <= value <= %g',field,lb,ub); 
end

function err = checkString(value,field,valid)
%Char array and valid string
err = [];
if(~ischar(value) || all(strcmpi(value,valid)==0))
    str = sprintf('\n');
    for i = 1:length(valid)
        str = sprintf('%s ''%s''\n',str,valid{i});
    end
    err = MException('jMPC:SetFieldError','Parameter ''%s'' should a string from the following supported options:\n%s',field,str);
end

function printfields()
%Print out fields with defaults

fprintf('\n MPC SETTINGS:\n');
fprintf('         LookAhead: [ Enable Setpoint Look Ahead (acausal), Off {0}, On (1) ]\n');
fprintf('       ScaleSystem: [ Automatically Scale MPC QP (Recommended), Off (0), On {1} ]\n');
fprintf('          ScaleFac: [ Normalized Scale Factor if Enabled Above {1} ]\n');
fprintf('            Single: [ Enable Single Precision MPC Controller (simulation only), Off {0}, On (1) ]\n');
fprintf('          InitialU: [ Vector Of Initial Control Inputs (at t=0), {[]} ]\n');

fprintf('\n QP SETTINGS:\n');
fprintf('          QPSolver: [ QP Solver (Empty = jMPC Heuristic), see list below {[]} ]\n');
fprintf('             QPTol: [ QP Solver Tolerance (Empty = Default: 1e-6 (double), 1e-4 (single)) {[]} ] \n');
fprintf('         QPMaxIter: [ Maximum QP Solver Iterations {30} ] \n');
fprintf('       QPWarmStart: [ Enable Warm Starting Heuristics (Selected Solvers), Off (0), On {1} ] \n');
fprintf('         QPVerbose: [ QP Solver Verbosity Level, Off {0}, Normal (1), High (2) ] \n');
fprintf('            QPSave: [ Save QP Problem at Specific Sample, {[]} ] \n');
fprintf('      QuadProgOpts: [ MATLAB''s quadprog optimset Options Structure {[]} ]\n');

fprintf('\n GENERAL SETTINGS:\n');
fprintf('          Warnings: [ Warning Level Control ''all'' or {''critical''} or ''none'' ]\n');
fprintf('           WaitBar: [ Enable Waitbar for Monitoring Simulation Progress, Off (0), On {1} ]\n');
fprintf('        InputNames: [ Cell Array of Control Input Names for Plotting {[]} ]\n');
fprintf('        InputUnits: [ Cell Array of Control Input Units for Plotting {[]} ]\n');
fprintf('       OutputNames: [ Cell Array of System Output Names for Plotting {[]} ]\n');
fprintf('       OutputUnits: [ Cell Array of System Output Units for Plotting {[]} ]\n\n');
fprintf('--------------------------------------------------------------------------------------------------');
fprintf('\n SUPPLIED QP SOLVERS (MEX Solvers on Windows Only):\n');  
fprintf('    ''quad_mehrotra'' - Mehrotra Predictor-Corrector [Double, MATLAB]\n');
fprintf('   ''mquad_mehrotra'' - Mehrotra Predictor-Corrector [Double, MEX]\n');
% fprintf('''mquad_mehrotraMKL'' - Mehrotra Predictor-Corrector [Double, MEX, Intel MKL (Large Problems)]\n');
fprintf('   ''squad_mehrotra'' - Mehrotra Predictor-Corrector [Single, MATLAB]\n');
fprintf('  ''msquad_mehrotra'' - Mehrotra Predictor-Corrector [Single, MEX]\n');
fprintf('      ''quad_wright'' - Wright Interior-Point [Double, MATLAB]\n');
fprintf('     ''mquad_wright'' - Wright Interior-Point [Double, MEX]\n');
% fprintf('  ''mquad_wrightMKL'' - Wright Interior-Point [Double, MEX, Intel MKL (Large Problems)]\n');
fprintf('     ''squad_wright'' - Wright Interior-Point [Single, MATLAB]\n');
fprintf('    ''msquad_wright'' - Wright Interior-Point [Single, MEX]\n');
fprintf('    ''quad_hildreth'' - Dual Search [Double, MATLAB]\n');

fprintf('\n INTERFACED QP SOLVERS:\n'); 
fprintf('         ''quadprog'' - Optimization Toolbox QP Solver [Double, MATLAB]\n');
fprintf('             ''qpip'' - QPC Interior Point [Double, MEX]\n');
fprintf('             ''qpas'' - QPC Active Set [Double, MEX]\n');
fprintf('              ''clp'' - OPTI Toolbox CLP Interior Point [Double, MEX]\n');
fprintf('             ''ooqp'' - OPTI Toolbox OOQP Interior Point [Double, MEX]\n');
fprintf('\n');




