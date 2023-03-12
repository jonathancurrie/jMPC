function sfun_scroll(block)
%Scrolling Simulink Display

%   Jonathan Currie (C)
%   Control Engineering 2011

setup(block);

function setup(block)

  % Register number of ports
  if(block.DialogPrm(5).Data)
    block.NumInputPorts  = 2;
    block.InputPort(2).DatatypeID  = 0;  % double
    block.InputPort(2).Complexity  = 'Real';
  else
    block.NumInputPorts = 1;
  end
  block.NumOutputPorts = 0;
  
  % Setup port properties to be inherited or dynamic
  block.SetPreCompInpPortInfoToDynamic;

  % Override input port properties
  block.InputPort(1).DatatypeID  = 0;  % double
  block.InputPort(1).Complexity  = 'Real';

  % Register parameters
  block.NumDialogPrms     = 5;

  %Collect Sample Time
  Ts = block.DialogPrm(1).Data;
  block.SampleTimes = [Ts 0];

  %Setup Options
  block.SetAccelRunOnTLC(false);
  block.SimStateCompliance = 'DefaultSimState';
  
  %Register Methods
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);
  block.RegBlockMethod('Outputs', @Outputs);
  block.RegBlockMethod('Update', @Update);
end  

    
function DoPostPropSetup(block)
  if(block.DialogPrm(5).Data) 
    block.NumDworks = 4;
  else
    block.NumDworks = 3;
  end
  block.Dwork(1).Name            = 'inBuffer';
  block.Dwork(1).Dimensions      = block.DialogPrm(2).Data*2;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'Counter';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'Ylim';
  block.Dwork(3).Dimensions      = 2;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;
  
  if(block.DialogPrm(5).Data)
      block.Dwork(4).Name            = 'Set_Point';
      block.Dwork(4).Dimensions      = block.DialogPrm(2).Data*2;
      block.Dwork(4).DatatypeID      = 0;      % double
      block.Dwork(4).Complexity      = 'Real';
      block.Dwork(4).UsedAsDiscState = true;
  end
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;
end

function InitConditions(block)
    % Initialize Dwork
    block.Dwork(1).Data = zeros(block.DialogPrm(2).Data*2,1);
    block.Dwork(2).Data = 1;
    block.Dwork(3).Data = block.DialogPrm(4).Data;
    if(block.DialogPrm(5).Data)
        block.Dwork(4).Data = zeros(block.DialogPrm(2).Data*2,1);
    end

    %Create Figure if one does not already exist
    FigHandle=get_param(gcb,'UserData');
    if (isempty(FigHandle) || ~ishandle(FigHandle))
        CreateSfunFigure(block);
    end

end

function Outputs(block)  
    %Check User hasn't deleted Figure
    FigHandle=get_param(gcb,'UserData');
    if ~ishandle(FigHandle),
        return;
    end

    Ts = block.DialogPrm(1).Data;
    Frame_Size = block.DialogPrm(2).Data;
    Count = block.Dwork(2).Data;
    f = 0;

    ud = get(FigHandle,'UserData');

    %Update Graphics Positions
    if(Count > 1)
        %Auto Scaling 
        y_lim = block.Dwork(3).Data;
        val = block.Dwork(1).Data(Count-1);
        if(val > y_lim(2)) 
            f = 1;               
            y_lim(2) = val + val*0.1; %could be improved using a difference equation
        else if(val < y_lim(1))
                f = 1;
                y_lim(1) = val + val*0.1;
            end
        end
        if(f)
            set(ud.AxHandle,'YLim',[y_lim(1) y_lim(2)]);
            block.Dwork(3).Data = y_lim;
        end
        set(ud.head,'xdata',(Count-1)*Ts,'ydata',block.Dwork(1).Data(Count-1));
        set(ud.tail,'xdata',(1:Count-1).*Ts,'ydata',block.Dwork(1).Data(1:Count-1));
        if(block.DialogPrm(5).Data)
            set(ud.setp,'xdata',(1:Count-1).*Ts,'ydata',block.Dwork(4).Data(1:Count-1));
        end
    end
    %Scrolling
    if(Count > Frame_Size)
        set(ud.AxHandle,'XLim',[(Count-Frame_Size)*Ts Frame_Size*Ts+(Count-Frame_Size)*Ts]);        
        %Reset Counter
        block.Dwork(2).Data = Frame_Size;
        %Shift Data
        block.Dwork(1).Data(1:Frame_Size) = block.Dwork(1).Data(2:Frame_Size+1);
        if(block.DialogPrm(5).Data)
            block.Dwork(4).Data(1:Frame_Size) = block.Dwork(4).Data(2:Frame_Size+1);
        end
    end
    %Draw
    drawnow; 
end

function Update(block)
    %Check User hasn't deleted Figure
    FigHandle=get_param(gcb,'UserData');
    if ~ishandle(FigHandle),
        return;
    end
    
    Count = block.Dwork(2).Data;
    %Update Input  
    block.Dwork(1).Data(Count) = block.InputPort(1).Data(1);
    if(block.DialogPrm(5).Data)
        block.Dwork(4).Data(Count) = block.InputPort(2).Data(1);
    end
    %Update Counter
    block.Dwork(2).Data = Count + 1;
    %Pause
    pause(block.DialogPrm(3).Data);   
end
 
%Create Figure
function FigHandle=CreateSfunFigure(block)
    
    %Create Figure
    FigHandle=figure('Name',get_param(gcb,'Name'),'NumberTitle','off','Visible','on');

    %Create Axes
    y_lim = block.DialogPrm(4).Data;
    ud.AxHandle = axes('Units','normalized','XLim',[0 block.DialogPrm(2).Data*block.DialogPrm(1).Data],...
                        'YLim',[y_lim(1) y_lim(2)],'Drawmode','fast','Visible','on','NextPlot','add','Xtick',[]);
    %Labels                
    xlabel('Time');
    ylabel('Amplitude');
    %Graphics
    ud.head = line('color','r','Marker','.','markersize',15,'xdata',[],'ydata',[]);
    ud.tail = line('color','b','LineStyle','-','xdata',[],'ydata',[]);
    if(block.DialogPrm(5).Data)
        ud.setp = line('color','k','LineStyle','--','xdata',[],'ydata',[]);
    end

    %Save objects into figure user data
    set(FigHandle,'HandleVisibility','on','UserData',ud);
    %Associate the figure with the block
    set_param(gcb,'UserData',FigHandle);
end  
    
end