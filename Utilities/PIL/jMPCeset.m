function options = jMPCeset(varargin)
%JMPCESET  Create or alter the options for jMPC Embedded Code Generator

% Print out possible values of properties.
if (nargin == 0) && (nargout == 0)
    printfields();
    return
end

Names = {'precision','strprecision','tbprecision','dir','verifyqp','verifympc','genbench','compareplot','arch','accelqp','verbose'}';
Defaults = {'match',[],'match',[],1,0,0,1,'generic',1,1}';         

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
    case {'compareplot','accelqp'}          
        err = scalar_int_lim(value,field,0,2); %0 <= val <= 2
    case {'verifyqp','verifympc','genbench','verbose'}
        err = scalar01(value,field);
    %Char
    case {'strprecision','dir'}
        if(~ischar(value))
            err = MException('jMPC:SetFieldError','Parameter ''%s'' should be a string',field);
        end 
    %Valid String
    case {'precision','tbprecision'}
        err = checkString(value,field,{'single','double','match'});
    case 'arch'
        err = checkString(value,field,{'generic','c2000','arm'});

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
%Print out fields with answers
fprintf('    precision: [ Generated MPC Controller Precision: ''match'' (match supplied controller''s precision), ''single'' or ''double'' ] \n');
fprintf(' strprecision: [ fprintf Precision: (e.g. %%1.10g, empty matches precision above) {[]} ] \n');
fprintf('  tbprecision: [ Testbench Precision for MEX validation: {''match''} (matches precision above), ''single'', ''double'' ] \n');
fprintf('     verifyqp: [ Generate MEX Testbench & Validate QP Code: {1} or 0 ] \n');
fprintf('    verifympc: [ Generate MEX Testbench & Validate MPC Code: 1 or {0} ] \n');
fprintf('  compareplot: [ Plot MEX verification results: 0, {1}, 2 ] \n');
fprintf('         arch: [ Processor Architecture: {''generic''}, ''arm'', ''c2000'' ] \n');
fprintf('      accelqp: [ Accelerate QP solver at the cost of extra memory: Off (0), Some {1}, Max (2) ]\n');
fprintf('      verbose: [ Command Window Output: {1} or 0 ] \n');

