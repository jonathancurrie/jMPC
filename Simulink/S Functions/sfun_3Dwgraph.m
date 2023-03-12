function sfun_3Dwgraph(block)
%Simulink 3D Animation Canvas + Customizable Scrolling Scopes

% Parts based on the MATLAB and HUMUSOFT demo 'Vehicle Dynamics
% Visualization with Graphs'

%   Jonathan Currie (C)
%   AUT University 2011 

    setup(block);
  
%endfunction

function setup(block)

    len = length(block.DialogPrm(2).Data);
    viden = block.DialogPrm(5).Data;

    % Register the number of ports.
    block.NumInputPorts  = len;
    if(viden)
        block.NumOutputPorts = 1;
        block.AllowSignalsWithMoreThan2D = 1;
    else
        block.NumOutputPorts = 0;
        block.AllowSignalsWithMoreThan2D = 0;
    end        

    % Set up the port properties to be inherited or dynamic.
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Override the input port properties.
    for i = 1:len
        block.InputPort(i).DatatypeID  = 0;  % double
        block.InputPort(i).Complexity  = 'Real';
        block.InputPort(i).DirectFeedthrough = false;
        block.InputPort(i).Dimensions = 1;
    end   
    if(viden)
        block.OutputPort(1).DatatypeID  = 3;  % uint8
        block.OutputPort(1).Complexity  = 'Real';
        block.OutputPort(1).Dimensions = [600 800 3];
    end

    % Register the parameters.
    block.NumDialogPrms  = 5;
    block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Nontunable','Nontunable'};

    % Set up the continuous states.
    block.NumContStates = 0;

    % Block Setup
    block.SampleTimes = [-1 0];    
    block.SetAccelRunOnTLC(false);
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods    
    block.RegBlockMethod('CheckParameters', @CheckPrms);
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
    block.RegBlockMethod('InitializeConditions', @InitConditions);
    block.RegBlockMethod('Update', @Update);
    block.RegBlockMethod('Outputs', @Outputs);
%end setup



function DoPostPropSetup(block)
    %Setup Work Vectors
    block.NumDworks = 1;

    block.Dwork(1).Name            = 'Counter';
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;      % double
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
    % Register all tunable parameters as runtime parameters.
    block.AutoRegRuntimePrms;
%end DoPostPropSetup  
 

function InitConditions(block)
    %Initialize Canvas & Figures
    FigHandle = GetCanvas(block);
    if ~ishandle(FigHandle)
      FigHandle = CreateCanvas(block); %Create a new one
    end
    
    %Reset counter
    block.Dwork(1).Data = 1;
    
    %Get Params
    ylims = block.DialogPrm(3).Data;
    TScroll = block.DialogPrm(4).Data;

    %Get UserData of the figure
    ud = get(FigHandle, 'UserData');
 
    %Loop over each line and axes setting them up
    len = length(ylims);
    for i = 1:len
        %Erase previously drawn lines
        set(ud.Line{i}, 'Erasemode', 'normal');
        %Clear Lines
        set(ud.Line{i}, 'XData', [], 'YData', []);
        %Draw first point, disable erasing
        set(ud.Line{i}, 'XData', 0, 'YData', 0, 'Erasemode', 'none');
        %Reset the Xlims and Ylims
        set(ud.Axes{i}, 'XLim', [0 TScroll], 'YLim', ylims{i});
        %Clear Y vector
        ud.YData{i} = [];
    end
    ud.XData = [];
    %Save user data
    set(FigHandle, 'UserData', ud);
%end InitConditions    


function Update(block)
    %Update Figure Plots
    %Get Figure Handle
    FigHandle = GetCanvas(block);
    if ~ishandle(FigHandle)
       return;
    end
    
    %Get Params
    len = length(block.DialogPrm(2).Data);
    TScroll = block.DialogPrm(4).Data;

    %Collect Counter value
    Count = block.Dwork(1).Data;
    %Update Counter
    block.Dwork(1).Data = Count + 1;
    % get UserData of the Figure
    ud = get(FigHandle, 'UserData');
    t = get(block,'CurrentTime');

    %Update Input Data Buffer 
    if (t <= TScroll)
        ud.XData(Count) = t;
        for i = 1:len
            ud.YData{i}(Count) = block.InputPort(i).Data(1);
        end
    else
       ud.XData = [ud.XData(2:end) t];
       for i = 1:len
           ud.YData{i} = [ud.YData{i}(2:end) block.InputPort(i).Data(1)];
           set(ud.Axes{i},'XLim',[ud.XData(1) ud.XData(end)]);
       end
    end

    %Plot the input lines
    for i = 1:len
        set(ud.Line{i},'Xdata', ud.XData,'Ydata', ud.YData{i}); 
    end
    %Store UserData
    set(FigHandle, 'UserData', ud);
    % Draw plots
    drawnow;
%end Update    


function Outputs(block)        
    %If Video Output Enabled
    if(block.DialogPrm(5).Data)
        %Get Figure Handle
        FigHandle = GetCanvas(block);
        if ~ishandle(FigHandle)
           return;
        end
        ud = get(FigHandle, 'UserData');
        %Get Current Frame
        fr = getframe(FigHandle);
        fr.cdata(ud.canvas_y1:ud.canvas_y2,ud.canvas_x1:ud.canvas_x2,:) = capture(ud.canvas);
        block.OutputPort(1).Data = fr.cdata;
    end
    

function CheckPrms(block)
    %Get Params
    titles = block.DialogPrm(2).Data;
    ylims = block.DialogPrm(3).Data;
    TScroll = block.DialogPrm(4).Data;
    
    if(~isa(titles,'cell'))
        error('You must supply a cell array of titles');
    end
    if(~isa(ylims,'cell'))
        error('You must supply a cell array of y limits');
    end
    if(~isa(TScroll,'double'))
        error('T limit must be a double');
    end
    
    lent = length(titles);
    leny = length(ylims);
    if(lent ~= leny)
        error('Your title and y limit cell arrays are not the same length!');
    end
    if(lent > 3)
        error('There is a maximum of 3 graphs available');
    end
    if(TScroll > 1e4 || TScroll < 1)
        error('The max and min t limit is 1e4 and 1 respectively');
    end
%end CheckPrms



%------ CUSTOM FUNCTIONS ------%
function FigHandle = GetCanvas(block)
%Get figure handle from block user data
if strcmp(get_param(block.BlockHandle, 'BlockType'), 'M-S-Function')
  block = get_param(block.BlockHandle, 'Parent');
end
FigHandle = get_param(block, 'UserData');
if isempty(FigHandle)
  FigHandle = -1;
end

function SetCanvas(block, FigHandle)
%Store figure handle in block user data
if strcmp(get_param(block.BlockHandle, 'BlockType'), 'M-S-Function')
block = get_param(block.BlockHandle, 'Parent');
end
set_param(block, 'UserData', FigHandle);

function FigHandle = CreateCanvas(block)
%Create Figure
width = 800; height = 600;
FigHandle = figure('Units',          'pixels', ...
                   'Position',       [100 100 width height], ...
                   'Color',          [0.314 0.314 0.314], ...
                   'Name',           'jMPC 3D Simulation',...%BlockFigureTitle(block), ...
                   'Tag',            'jMPC_Sim3D_Canvas', ...
                   'NumberTitle',    'off', ...
                   'IntegerHandle',  'off', ...
                   'Toolbar',        'none');

%Store the block handle in the figure UserData
ud.Block = block;

%Get User Params
vrw = block.DialogPrm(1).Data;
titles = block.DialogPrm(2).Data;
len = length(titles);
switch(len)
    case 1
        pos{1} = [0.05 0.05 0.9 0.25];
    case 2
        pos{1} = [0.05 0.05 0.42 0.25];
        pos{2} = [0.54 0.05 0.42 0.25];
    case 3
        pos{1} = [0.05 0.05 0.28 0.25];
        pos{2} = [0.37 0.05 0.28 0.25];
        pos{3} = [0.69 0.05 0.28 0.25];
end

%Loop creating axes and setting them up as we go
for i = 1:len
    ud.Axes{i} = axes('Position', pos{i}, ...
                  'XGrid',    'off', ...
                  'YGrid',    'on', ...
                  'Color',    'k', ...
                  'XColor',   'w', ...
                  'YColor',   'w', ...
                  'XTick',    [],...
                  'Drawmode', 'fast');
    set(ud.Axes{i}, 'Title', title(titles{i}, 'Color', 'w'));    
    ud.Line{i} = line(0, 0, 'EraseMode', 'None', 'Color', 'y', 'LineStyle', '-');  
end

% open vrworld if not open already
vr_world = vrworld(vrw);
if ~isopen(vr_world)
  open(vr_world);
end
ud.vr_world = vr_world;

% create canvas in the figure
ud.canvas = vr.canvas(vr_world, 'Parent', FigHandle, ...
          'Units', 'normalized', ...
          'Position', [0.03 0.36 0.93 0.63]);  
% create index positions
ud.canvas_x1 = floor((width-0.93*width)/2);
ud.canvas_x2 = ud.canvas_x1+0.93*width-1;
ud.canvas_y1 = 18;
ud.canvas_y2 = ud.canvas_y1+0.63*height-1;

% Associate the figure with the block, and set the figure's UserData.
SetCanvas(block, FigHandle);
set(FigHandle, 'UserData', ud, 'HandleVisibility', 'callback');
