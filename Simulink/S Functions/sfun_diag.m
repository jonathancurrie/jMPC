function sfun_diag(block)
%Diagonalize Vector

setup(block);

function setup(block)
    % Register number of ports
    block.NumInputPorts = 1;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;

    % Override input port properties
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DimensionsMode    = 'Fixed';
    block.InputPort(1).DirectFeedthrough = true;   
    
    % Register parameters
    block.NumDialogPrms  = 0;

    %Sample Time
    block.SampleTimes = [-1 0];
    
    %Setup
    block.SimStateCompliance = 'DefaultSimState';
    block.SetAccelRunOnTLC(false);

    %Register Methods
    block.RegBlockMethod('SetInputPortDimensions', @SetInputPortDims);
    block.RegBlockMethod('Outputs', @Outputs);
end  

function SetInputDimsMode(block, port, dm)
% Set dimension mode
block.InputPort(port).DimensionsMode = dm;
end

function SetInputPortDims(block, idx, di)
% Set compiled dimensions 
block.InputPort(idx).Dimensions = di;
block.OutputPort(idx).Dimensions = [di di];
end

function Outputs(block)              
    block.OutputPort(1).Data = diag(block.InputPort(1).Data);
end 
    
end