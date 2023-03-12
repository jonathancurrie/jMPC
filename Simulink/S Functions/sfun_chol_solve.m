function sfun_chol_solve(block)
%Cholesky Linear Solver

setup(block);

function setup(block)
    % Register number of ports
    block.NumInputPorts = 2;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    %block.SetPreCompOutPortInfoToDynamic;

    % Override input port properties
    block.InputPort(1).DatatypeID  = 0;  % double
    block.InputPort(1).Complexity  = 'Real';
    block.InputPort(1).DimensionsMode    = 'Fixed';
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
    block.InputPort(2).DimensionsMode    = 'Fixed';
    block.InputPort(2).DirectFeedthrough = true;    
    
    % Register parameters
    block.NumDialogPrms  = 1;

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
if(idx == 2)
    block.OutputPort(1).Dimensions = di;
end
end

function Outputs(block)              
    [R,e] = chol(block.InputPort(1).Data);
    if(e > 0)
        if(block.DialogPrm(1).Data == 2)
            warning('Input Matrix H is not positive definite!'); %#ok<WNTAG>
        elseif(block.DialogPrm(1).Data == 3)
            error('Input Matrix H is not positive definite!');
        end
        block.OutputPort(1).Data = block.InputPort(2).Data;
    else
        %Linsolve options
        opU.UT = true; opUT.UT = true; opUT.TRANSA = true;
        block.OutputPort(1).Data = linsolve (R, linsolve (R, block.InputPort(2).Data, opUT), opU);
    end
end 
    
end