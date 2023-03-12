function sfun_jNL(block)
% Nonlinear Model S Function

%   Jonathan Currie (C)
%   AUT University 2011    

    setup(block);

function setup(block)  
    % Register number of dialog parameters   
    block.NumDialogPrms = 1;

    % Register number of input and output ports
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 2;
    
    % Setup functional port properties to dynamically inherited.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;
    % Override defaults
    block.InputPort(1).DatatypeID        = 0;  % double
    block.InputPort(1).Complexity        = 'Real';
    block.InputPort(1).DimensionsMode    = 'Fixed';
    block.InputPort(1).DirectFeedthrough = false; %otherwise causes algebraic loop
    
    %Determine Sizes
    if(isa(block.DialogPrm(1).Data.C,'function_handle'))
        states = length(block.DialogPrm(1).Data.x0);
        temp = block.DialogPrm(1).Data.C(0,block.DialogPrm(1).Data.x0,0,block.DialogPrm(1).Data.param); %dummy call
        nout = length(temp);
    else    
        [nout,states] = size(block.DialogPrm(1).Data.C);
    end
    block.OutputPort(1).Dimensions       = nout;
    block.OutputPort(1).SamplingMode     = 0;  %sample based
    block.OutputPort(2).Dimensions       = states;
    block.OutputPort(2).SamplingMode     = 0;
  
    % Set block sample time to continuous
    block.SampleTimes = [0 0];

    % Setup Dwork
    block.NumContStates = size(block.DialogPrm(1).Data.x0,1);

    % Set the block simStateCompliance to default (i.e., same as a built-in block)
    block.SimStateCompliance = 'DefaultSimState';
    block.SetAccelRunOnTLC(false);

    % Register methods    
    block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSampMode);
    block.RegBlockMethod('SetInputPortDimensions',  @SetInputPortDims);
    block.RegBlockMethod('InitializeConditions',    @InitConditions); 
    block.RegBlockMethod('Outputs',                 @Output);  
    block.RegBlockMethod('Derivatives',             @Derivative);  
end

function SetInputDimsMode(block, port, dm)
    % Set dimension mode
    block.InputPort(port).DimensionsMode = dm;
end

function SetInputPortSampMode(block, port, mode)
    % Set sampling mode
    block.InputPort(port).SamplingMode = mode;   
end

function SetInputPortDims(block, idx, di)
    % Set compiled dimensions 
    block.InputPort(idx).Dimensions = di;
end

function InitConditions(block)
    % set x0
    block.ContStates.Data = block.DialogPrm(1).Data.x0;
end

function Output(block)
    model = block.DialogPrm(1).Data;
    x = block.ContStates.Data;
    
    if(isa(model.C,'function_handle'))
        %y = C(x) NOTE U CANNOT BE USED AS IT IS NOT FED THROUGH
        block.OutputPort(1).Data = model.C(0,x,0,model.param);
    else    
        % y = C * x
        block.OutputPort(1).Data = model.C * x;
    end
    block.OutputPort(2).Data = x;
end

function Derivative(block)
    %Collect Args
    u =  block.InputPort(1).Data;
    model = block.DialogPrm(1).Data;
    %Compute Derivatives
    block.Derivatives.Data = model.nl_model([],block.ContStates.Data,u,model.param);
end

end