classdef ScrollPlot
%ScrollPlot  Create a Scrolling Plot Object
%
%   SPobj = ScrollPlot(Mode,Ts,Frame_Size,Np,Update_Delay,y_lim,no_lines)
%   creates a ScrollPlot object, with Mode specifying the following:
%
%       1 -  1-4 Scrolling Lines
%       2 -  Mode 1 + Constant Horizontal Constraint Lines Added
%       3 -  Mode 2 + Scrolling Stair Plot of Setpoint Lines
%
%   Ts specifies the sampling period of the display. Frame Size is the
%   number of samples in the 'Present' plot, while Np is the number of
%   samples in the 'Prediction' plot. Update_Delay is used for standalone
%   operation, and is used by 'pause' to slow down plotting. y_lim
%   specifies the initial y-axis limits, and no_lines specifies the number
%   of lines to plot (1 - 4).
%
%   SPobj = ScrollPlot(Mode,...,no_lines,AxesHandles) plots the scrolling
%   plots into the axes specified by AxesHandle. This should be a two
%   element vector containing axes handles. When omitted, the object will
%   create a new figure (or if an existing ScrollPlot figure exists, clear
%   it and draw in that).
%
%   SPobj = ScrollPlot(Mode,...,AxesHandles,Constraints) creates static
%   horizontal dotted lines representing upper and lower constraints.
%   Constraints is a matrix, where column one are the lower constraints,
%   and column two are the upper constraints.
%
%   See also ScrollPlot.plot ScrollPlot.stairs

%   Jonathan Currie (C)
%   Control Engineering 2009
    
    properties(SetAccess = private)
        Mode                % Drawing Mode 1 = line only, 2 = +constraints, 3 = +setpoint
        Ts                  % Sampling Period
        Frame_Size          % Present Window Frame Size (number of samples)
        Np                  % Prediction Window Frame size (number of samples)
        Update_Delay        % Pause Delay when used stand alone
        y_lim               % Starting Limits of y axis
        no_lines            % Number of lines to draw
        setp_lines          % Number of setpoint lines
        AxesHandle          % Handles to the drawing axes
        Graphics            % Handles to the lines & points
    end
    
    properties(SetAccess = public)
        Rescale             % Allows the window to resize smaller when at steady state
    end
    
    properties(SetAccess = private, GetAccess = private)
        y_buf               % Buffer containing the present window's data 
        setp_buf            % Buffer containing the present window's setpoint data
        Count               % Internal counter to determine position
        Ratio               % Screen size ratio
    end
    
    %Class Only Methods
    methods
        %-- Constructor --%
        function S = ScrollPlot(Mode,Ts,Frame_Size,Np,Update_Delay,y_lim,no_lines,varargin)
            %Check Optional Inputs
            if(~isempty(varargin)); S.AxesHandle = varargin{1}; else S.AxesHandle = [-1 -1]; end         
            
            %Check inputs
            if(iscell(no_lines))
                S.setp_lines = no_lines{2};
                no_lines = no_lines{1};
            else
                S.setp_lines = no_lines;
            end
                
            if(Frame_Size/Np < 2)
                error('Frame Size must be at least 2x greater than Np');
            end
            if(no_lines > 4)
                error('Maximum 4 I/O Supported');
            end  
            
            %Create Figure & Axes if not supplied by user
            if(~ishandle(S.AxesHandle(1)))
                %Check if a scrolling plot already exists         
                fig = get(0,'CurrentFigure');
                if(isempty(fig) || ~strcmp(get(fig,'Name'),'Scrolling Window')) %rough method!
                    %Create Figure
                    figure('Name','Scrolling Window','NumberTitle','off','Visible','on','Position',[380 320 800 420]);
                else
                    clf;
                end
                %Create Axes
                S.AxesHandle(1) = axes('Units','normalized','XLim',[0 Ts*Frame_Size],'YLim',y_lim...
                                ,'Drawmode','fast','Visible','on','Xtick',[],'Position',[0.08 0.08 0.65 0.85]);
                ylabel('Amplitude');
                xlabel('Time');
                title('Present');
                S.AxesHandle(2) = axes('Units','normalized','XLim',[0 Ts*Np],'YLim',y_lim...
                                ,'Drawmode','fast','Visible','on','Xtick',[],'Ytick',[],...
                                'Position',[0.733 0.08 0.25 0.85]);
                title('Prediction');
            end
            
            %Calculate Aspect Ratio (needs work)
            asRatio = Ts*Frame_Size/(Frame_Size-Np)*0.5;          
            %Create Colour Array
            colour =  {[0 0 1],[0.2 0.5 0.2],[0.8 0.2 0.2],[0.44 0.2 0.56]};
            setpcolour = {[0.5 0.5 1],[0.4 0.8 0.4],[1 0.52 0.52],[0.74 0.48 0.69]};
            %Create Present Graphics  
            axes(S.AxesHandle(1)); %#ok<MAXES>
            for i = 1:no_lines
                S.Graphics(1).Head(i) = line('color','r','Marker','.','markersize',15,'xdata',[],'ydata',[]);
                S.Graphics(1).Tail(i) = line('color',colour{i},'LineStyle','-','xdata',[],'ydata',[]);
            end
            %Create Prediction Graphics
            axes(S.AxesHandle(2)); %#ok<MAXES>
            for i = 1:no_lines
                S.Graphics(2).Head(i) = line('color','r','Marker','o','markersize',5,'xdata',[],'ydata',[]);
                S.Graphics(2).Tail(i) = line('color',colour{i},'LineStyle',':','xdata',[],'ydata',[]);
            end
            %Create Optional Graphics
            if(Mode > 1)
                axes(S.AxesHandle(1)); %#ok<MAXES>
                %Create Constant Constraint Lines
                x = [0 Ts*Frame_Size];
                ymin = varargin{2}(:,1)';
                ymax = varargin{2}(:,2)';
                %Create Constraint Line Objects
                for i = 1:no_lines
                    if(ymax(i) >= 1e6); ymax = NaN*ones(length(ymax),1); end
                    if(ymin(i) <= -1e6); ymin = NaN*ones(length(ymin),1); end
                    S.Graphics(1).consmax(i) = line('color',setpcolour{i},'LineStyle',':','xdata',x,'ydata',ones(length(x),1)*ymax(i)); %[1 0.7 0]
                    S.Graphics(1).consmin(i) = line('color',setpcolour{i},'LineStyle',':','xdata',x,'ydata',ones(length(x),1)*ymin(i)); %[1 0.8 0]
                end
                %Create Setpoint if required
                if(Mode > 2)
                    S.setp_buf = zeros(Frame_Size+1,S.setp_lines);
                    for i = 1:S.setp_lines
                        S.Graphics(1).setp(i) = line('color',setpcolour{i},'LineStyle','-.','xdata',[],'ydata',[]);
                    end
                end
                axes(S.AxesHandle(2)); %#ok<MAXES> (move to front)
            end
            
            %Save Class Properties
            S.Ts = Ts;
            S.Frame_Size = Frame_Size;
            S.Np = Np;
            S.Update_Delay = Update_Delay;
            S.y_buf = zeros(Frame_Size+1,no_lines); 
            S.y_lim = y_lim;
            S.Count = 1;
            S.Ratio = asRatio;
            S.Mode = Mode;
            S.no_lines = no_lines;
            S.Rescale.on = 0;
        end
        
        %-- Display --%
        function disp(S)
            ModeStr = {'Normal','Normal + Constraints','Normal + Setpoint + Constraints'};
            fprintf('-- ScrollPlot Object --\n')
            fprintf('Frame Size:       %1d samples\n',S.Frame_Size); 
            fprintf('Prediction Size:  %1d samples\n',S.Np);
            fprintf('Sample Time:      %1.2g s\n',S.Ts);
            fprintf('Update Delay:     %1.2g s\n',S.Update_Delay);
            fprintf('Mode:             %s\n\n',ModeStr{S.Mode});
        end
        
        %-- Update Plot --%
        function S = plot(S,Resize,y,pred,varargin)  
            %PLOT  Update the Scrolling Plot with New Data
            %
            %   SPobj = plot(SPobj,Resize,y,pred) draws the data contained
            %   in y onto the current ScrollPlot 'Present' axes, and the 
            %   data contained in pred into the 'Prediction' axes. Resize
            %   is a dynamic flag which when 1, allows the Scrolling Window
            %   to rescale smaller.
            %
            %   y should be a row vector, and pred a matrix
            %
            %   SPobj = plot(SPobj,Resize,y,pred,setp) also draws the data
            %   in setp onto the present axes.
            %
            %   See also plot ScrollPlot.stairs
            
            %Update Scrolling Position
            S = scroll(S,Resize,0,y,pred,varargin{:});
            %Pause & Draw
            pause(S.Update_Delay);            
            drawnow;
        end
        
        %-- Update Stairs Plot --%
        function S = stairs(S,Resize,y,pred) 
            %STAIRS  Update the Scrolling Plot with New Data
            %
            %   SPobj = stairs(SPobj,Resize,y,pred) draws using a ZOH the 
            %   data contained in y onto the current ScrollPlot 'Present' 
            %   axes, and the data contained in pred into the 'Prediction' 
            %   axes. Resize is a dynamic flag which when 1, allows the 
            %   Scrolling Window to rescale smaller.
            %
            %   y should be a row vector, and pred a matrix
            %
            %
            %   See also stairs ScrollPlot.plot
            
            %Update Scrolling Position
            S = scroll(S,Resize,1,y,pred);
            %Pause & Draw
            pause(S.Update_Delay);            
            drawnow;
        end
        
        %-- Scroll The Image --%
        function S = scroll(S,Resize,Mode,y,pred,varargin)
            % Scrolls the axes - not to be called by a user
            
            %Update Positions
            if(S.Count > 1)
                %Auto Scaling (Increase Plot Size)
                f = 0;
                if(S.Mode > 2); 
                    setp = varargin{1}; 
                else
                    setp = []; 
                end;
                maxval = max([max(max([S.y_buf(1:end-2,:);y])) setp]);
                minval = min([min(min([S.y_buf(1:end-2,:);y])) setp]);
                if(maxval > S.y_lim(2)) 
                    f = 1;               
                    S.y_lim(2) = maxval + abs(maxval*0.1); %could be improved using a difference equation
                else if(minval < S.y_lim(1))
                        f = 1;
                        S.y_lim(1) = minval - abs(minval*0.1);
                    end
                end
                if(f)
                    set(S.AxesHandle(1),'YLim',S.y_lim);
                    set(S.AxesHandle(2),'YLim',S.y_lim);
                end
                %Present Positions
                for i = 1:S.no_lines
                    if(Mode)
                        [x1,y1] = stairs((S.Count-1)*S.Ts,S.y_buf(S.Count-1,i));
                        [x2,y2] = stairs((1:S.Count-1).*S.Ts, S.y_buf(1:S.Count-1,i));
                    else
                        x1 = (S.Count-1)*S.Ts;
                        y1 = S.y_buf(S.Count-1,i);
                        x2 =(1:S.Count-1).*S.Ts;
                        y2 = S.y_buf(1:S.Count-1,i);
                    end
                    set(S.Graphics(1).Head(i),'xdata',x1,'ydata',y1);
                    set(S.Graphics(1).Tail(i),'xdata',x2,'ydata',y2); 
                    if((S.Mode > 2) && (i <= S.setp_lines))
                        set(S.Graphics(1).setp(i),'xdata',x2,'ydata',S.setp_buf(1:S.Count-1,i));
                    end
                end
            end
            %Scrolling
            if(S.Count > S.Frame_Size)
                for i = 1:S.no_lines
                    set(S.AxesHandle(1),'XLim',[(S.Count-S.Frame_Size)*S.Ts S.Frame_Size*S.Ts+(S.Count-S.Frame_Size)*S.Ts]);
                    S.Count = S.Frame_Size;
                     %Shift Data
                    S.y_buf(1:S.Frame_Size,i) = S.y_buf(2:S.Frame_Size+1,i);
                    if((S.Mode > 2) && (i <= S.setp_lines))
                        S.setp_buf(1:S.Frame_Size,i) = S.setp_buf(2:S.Frame_Size+1,i);
                    end
                end
                if(Resize)
                    %Resize if at steady state
                    if(abs(S.y_buf(1,:)-mean((S.y_buf(1:S.Frame_Size-1,:)))) < 1e-3) %assume steady state                        
                            if(~S.Rescale.on)
                                S.Rescale.on = 1;
                                S.Rescale.delay = -S.Frame_Size/2;
                                %Setup Target limits and step size
                               if((minval < 0.01) && (minval > -0.01)) %if around 0
                                    minval = -0.1;
                                end
                                if((maxval < 0.01) && (maxval > -0.01))
                                    maxval = 0.1;
                                end
                                S.y_lim(1) = roundn(S.y_lim(1),-3);
                                S.y_lim(2) = roundn(S.y_lim(2),-3);                        
                                S.Rescale.y2 = roundn(maxval + abs(maxval*0.1),-3);
                                S.Rescale.y2s = roundn(abs(S.y_lim(2) - S.Rescale.y2)/50,-3);
                                S.Rescale.y1 = roundn(minval - abs(minval*0.1),-3);
                                S.Rescale.y1s = roundn(abs(S.y_lim(1) - S.Rescale.y1)/50,-3);
                            end
                            if(S.Rescale.delay >= 0) %delay the start of rescaling
                                if((S.y_lim(2) - S.Rescale.y2s) > S.Rescale.y2)
                                    S.y_lim(2) = S.y_lim(2) - S.Rescale.y2s;
                                end
                                if((S.y_lim(1) + S.Rescale.y1s) < S.Rescale.y1)
                                    S.y_lim(1) = S.y_lim(1) + S.Rescale.y1s;
                                end
                                set(S.AxesHandle(1),'YLim',S.y_lim);
                                set(S.AxesHandle(2),'YLim',S.y_lim);
                            else
                                S.Rescale.delay = S.Rescale.delay + 1;
                            end
                    else
                        S.Rescale.on = 0;
                    end
                else
                    S.Rescale.on = 0;
                end
            end
            %Update Prediction Positions
            for i = 1:S.no_lines
                if(Mode)
                    [x1,y1] = stairs((S.Np-1)*S.Ratio,pred(end,i));
                    [x2,y2] = stairs(0:S.Ratio:(S.Np-1)*S.Ratio, pred(:,i));
                else
                    x1 = (S.Np-1)*S.Ratio;
                    y1 = pred(end,i);
                    x2 = 0:S.Ratio:(S.Np-1)*S.Ratio;
                    y2 = pred(:,i);
                end
                set(S.Graphics(2).Head(i),'xdata',x1,'ydata',y1);
                set(S.Graphics(2).Tail(i),'xdata',x2,'ydata',y2); 
            end
            %Insert Recent Sample
            S.y_buf(S.Count,:) = y;
            if(S.Mode > 2)
                S.setp_buf(S.Count,:) = varargin{1};
            end   
            %Update Counter
            S.Count = S.Count + 1;
        end 
    end
end

function x = roundn(x,n)
%ROUNDN  Round numbers to specified power of 10

% Copyright 1996-2007 The MathWorks, Inc.
% $Revision: 1.9.4.4 $    $Date: 2007/11/09 20:25:18 $
% Written by:  E. Byrns, E. Brown

    %  Compute the exponential factors for rounding at specified
    %  power of 10.  Ensure that n is an integer.
    factors  = 10 ^ (fix(-n));
    %  Set the significant digits for the input data
    x = round(x * factors) / factors;
end
