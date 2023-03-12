function varargout = jMPC_GUI(varargin)
%Creates the Main GUI for Simulating a jMPC Controller & Plant

% Jonathan Currie (C)
% AUT University 2011 

% Please note this GUI uses tabpanel 2.8.1 by Elmar Tarajan - covered by a
% BSD License.

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @jMPC_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @jMPC_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code 


%%--------------- OPENING FUNCTION ---------------%%
%Setup axes and defaults
function jMPC_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
global colour guiver;

% Choose default command line output for jMPC_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Turn off Singular Warnings
warning off MATLAB:nearlySingularMatrix

%Check Screen Size
scrn = get(0,'ScreenSize');
if((scrn(3) < 1200) || (scrn(4) < 750))
    error('MSGID:jMPC_ScreenSize','jMPC GUI Cannot Be Displayed In Resolutions Less than 1200x750')
end

%My Setup
clc
guiver = jMPCver();
set(gcf,'Name','jMPC Simulation Tool');
%Setup Axes
axes(handles.outPres); cla; title('Past & Present Outputs'); xlabel('Time'); ylabel('Amplitude')   %#ok<MAXES>
axes(handles.inPres); cla; title('Past & Present Inputs'); xlabel('Time'); ylabel('Amplitude') %#ok<MAXES>
axes(handles.outPred); cla; title('Predicted Outputs'); %#ok<MAXES>
axes(handles.inPred); cla; title('Future Inputs') %#ok<MAXES>
%Setup GUI Objects
set(handles.pause_button,'Enable','off');
set(handles.sim_panel,'UserData',0);
set(handles.model_panel,'UserData',0);
set(handles.setpoint_popup,'ForegroundColor','b');
set(handles.load_check,'Value',0);
set(handles.load_slide,'Enable','off');
set(handles.load_text,'Enable','off');
set(handles.load_text_top,'Enable','off');
set(handles.load_popup,'Enable','off');
set(handles.load_popup,'ForegroundColor','b');
set(handles.noise_check,'Value',0);
set(handles.noise_slide,'Enable','off');
set(handles.noise_text,'Enable','off');
set(handles.noise_text_top,'Enable','off');
set(handles.noise_popup,'Enable','off');
set(handles.noise_popup,'ForegroundColor','b');
set(handles.state_est_check,'Value',1);
set(handles.state_est_slide,'Value',0.5);
set(handles.state_est_text,'UserData',0.5);
set(handles.state_est_text,'String',num2str(0.5));
set(handles.frame_popup,'Value',4);
set(handles.soft_check,'Value',0);
set(handles.out_led,'XLim',[-1 1],'YLim',[-1 1]...
     ,'Drawmode','fast','Visible','on','Xtick',[],'Ytick',[]...
     ,'Box','on','color',[0.878,0.875,0.89]);
 set(handles.in_led,'XLim',[-1 1],'YLim',[-1 1]...
     ,'Drawmode','fast','Visible','on','Xtick',[],'Ytick',[]...
     ,'Box','on','color',[0.878,0.875,0.89]);
%axes(handles.in_led); cla; %#ok<MAXES>
status = 'Stopped';
update_console(handles,status);
set(handles.version_text,'String',sprintf('Version %2.2f  ',guiver));
%Set Available QP Solvers
avSolvers = {'jMPC Mehrotra','jMPC Wright'};
if(exist('quadprog','file')==2), avSolvers = [avSolvers 'quadprog']; end
if(exist('qpip','file')==3), avSolvers = [avSolvers 'QPC qpip']; end
if(exist('qpas','file')==3), avSolvers = [avSolvers 'QPC qpas']; end
if(exist('ooqp','file')==3), avSolvers = [avSolvers 'OOQP']; end
if(exist('clp','file')==3), avSolvers = [avSolvers 'CLP']; end
set(handles.qp_popup,'String',avSolvers);
set(handles.qp_popup,'Value',1);
%Save User Data
Ts = 0.1; %default sample time
set(handles.sample_edit,'UserData',Ts);
set(handles.start_button,'UserData',status);
%Refresh Variables
refresh_button_Callback(hObject, eventdata, handles);
%Create Colour Array
colour = {'b',[0.2 0.6 0.2],[0.8 0.2 0.2],[0.4 0.2 0.3]};
%end jMPC_GUI_OpeningFcn

%%--------------- OUTPUT FUNCTION ---------------%%
%Not used
function varargout = jMPC_GUI_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>
varargout{1} = handles.output;
%end jMPC_GUI_OutputFcn

%%--------------- START BUTTON ---------------%%
%Start MPC simulation with given settings
function start_button_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
status = get(handles.start_button,'UserData');
if(strcmp(status,'Stopped'))    
    ok = start_sim(handles);
    if(ok)
        tabselectionfcn(gcf,'Tabs',3);
    end    
else
    stop_sim(handles);
    tabselectionfcn(gcf,'Tabs',1);
    %If Data logging Selected
    if(strcmp(get(handles.data_log_menu,'Checked'),'on'))
        %Collect User Data
        plotvec = get(handles.data_log_menu,'UserData');
        Plant = get(handles.plant_list,'UserData');
        MPC1 = get(handles.top_panel,'UserData');
        Ts = get(handles.sample_edit,'UserData');
        T = length(plotvec.u)-1;
        %Build jSIMopt object
        simresult = jSIM(MPC1,Plant,T,Ts,plotvec.setp);
        simresult.plotvec = plotvec;
        simresult.opts.result = 1;
        %Export variables to workspace
        assignin('base','plotvec',plotvec);
        assignin('base','simresult',simresult);
        assignin('base','MPC1',MPC1);
        %Display Command
        update_console(handles,sprintf('To View The Full Simulation Type:\n   plot(MPC1,simresult)'));
    end
end

%%--------------- PAUSE BUTTON ---------------%%
%Pause simulation
function pause_button_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Collect User Data
status = get(handles.start_button,'UserData');
%Clear Messages
update_console(handles,[]);
if(strcmp(status,'Paused'))
    set(handles.start_button,'Enable','on');
    status = 'Running';
    sim_timer = get(handles.pause_button,'UserData');
    start(sim_timer);
else
    status = 'Paused';
    set(handles.start_button,'Enable','off');
    sim_timer = get(handles.pause_button,'UserData');
    stop(sim_timer);
end
update_console(handles,status);
%Save User Data
set(handles.start_button,'UserData',status);
%end pause_button_Callback


%%--------------- AUTO LOAD BUTTON ---------------%%
%Automatically load variables from the base workspace
function auto_button_Callback(hObject, eventdata, handles)  %#ok<DEFNU>
%Clear Messages
update_console(handles,[]);
%Refresh Variables
refresh_button_Callback(hObject, eventdata, handles);
%Collect User Data
Model = get(handles.model_list,'UserData');
Plant = get(handles.plant_list,'UserData');
if(isempty(Model) || isempty(Plant)) 
    update_console(handles,'A Plant & Model Must Be Selected to Use This Feature');
    return;
end
%Collect data from workspace using default names
vars = {'Np','Nc','uwt','ywt','con','setp','Ts'};
j = 1;
for i = 1:length(vars)
    try 
         col{i} = evalin('base',vars{i}); %#ok<AGROW>
    catch %#ok<CTCH>
        errs{j} = vars{i}; %#ok<AGROW>
        j = j + 1;
    end
end
if(j > 1)
    update_console(handles,sprintf('Auto Complete Error: \n'));
    for i = 1:length(errs)
        update_console(handles,sprintf('Check the Variable %s Exists',errs{i}));
    end
    return
end

%Write Data to GUI
write_GUI(handles,col{1},col{2},col{3},col{4},col{5},col{6},col{7});
update_console(handles,'Auto Load Complete');
%end auto_button_Callback

%%--------------- DEFAULTS BUTTON ---------------%%
%Setup default options
function default_button_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Collect User Data
Model = get(handles.model_list,'UserData');
Plant = get(handles.plant_list,'UserData');
%Clear Messages
update_console(handles,[]);
%Collect Model & Plant
model_list_Callback(handles.model_list, eventdata, handles)
plant_list_Callback(handles.plant_list, eventdata, handles)
if(isempty(Model) || isempty(Plant)) 
    update_console(handles,'A Plant & Model Must Be Selected to Use This Feature');
    return;
end
n_in = size(Model.B,2);
n_out = size(Model.C,1);
n = max([n_in n_out]);
%Setup GUI Objects with Default settings based on Model Size
set(handles.sample_edit,'String','0.1');
set(handles.prediction_edit,'String','10');
set(handles.control_edit,'String','5');
set(handles.state_est_check,'Value',1);
set(handles.state_est_slide,'Value',0.5);
set(handles.qp_popup,'Value',3);
set(handles.soft_check,'Value',0);
set(handles.load_check,'Value',0);
set(handles.load_slide,'Enable','off');
set(handles.load_popup,'Enable','off');
set(handles.load_text_top,'Enable','off');
set(handles.load_text,'Enable','off');
set(handles.noise_check,'Value',0);
set(handles.noise_slide,'Enable','off');
set(handles.noise_popup,'Enable','off');
set(handles.noise_text_top,'Enable','off');
set(handles.noise_text,'Enable','off');
set(handles.setpoint_popup,'Value',1);
set(handles.setpoints_table,'Data',zeros(n_out,1));
setpoints = zeros(n_out,1);
set(handles.setpoint_slide,'Value',0);
%Create weights & constraints
uwt = ones(n_in,1);
ywt = ones(n_out,1);
ucon = repmat([-5 5 2.5],n_in,1);
ycon = repmat([-5 5],n_out,1);
if(n_in ~= n_out)
    uwt = [uwt; zeros(n-n_in,1)];
    ywt = [ywt; zeros(n-n_out,1)];
    ucon = [ucon; zeros(n-n_in,3)];
    ycon = [ycon; zeros(n-n_out,2)];
end
set(handles.weights_table,'Data',[uwt ywt]);
set(handles.constraints_table,'Data',[ucon ycon]);
%Save User Data
set(handles.setpoint_popup,'UserData',setpoints);
%end default_button_Callback


%%--------------- REFRESH BUTTON ---------------%%
%Read the workspace and reload the plant and model
function refresh_button_Callback(hObject, eventdata, handles) %#ok<INUSL>
%Clear Messages
update_console(handles,[]);
%Reload workspace
read_workspace(handles);
%Reload Plant & Model
load_model(handles);
load_plant(handles);
update_console(handles,'Refresh Complete');
%end refresh_button_Callback


%%--------------- SIMPLE CALLBACKS ---------------%%
% --- Executes on selection change in plant_list.
function plant_list_Callback(hObject, eventdata, handles) %#ok<INUSL>
%Load Plant from List
load_plant(handles);
%end plant_list_Callback

% --- Executes on selection change in model_list.
function model_list_Callback(hObject, eventdata, handles) %#ok<INUSL>
%Load Model From List
load_model(handles);
%end model_list_Callback

% --- Executes on slider movement.
function setpoint_slide_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Collect setpoint vector
setpoints = get(handles.setpoint_popup,'UserData');
ind = get(handles.setpoint_popup,'Value');
setpoints(ind) = get(handles.setpoint_slide,'Value');
set(handles.setpoint_popup,'UserData',setpoints);
%end setpoint_slide_Callback

% --- Executes on selection change in setpoint_popup.
function setpoint_popup_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Update slider position based on setpoint selected
global colour;
%Collect setpoint vector
setpoints = get(handles.setpoint_popup,'UserData');
ind = get(hObject,'Value');
set(handles.setpoint_slide,'Value',setpoints(ind));
set(handles.setpoint_popup,'ForegroundColor',colour{ind});
%end setpoint_popup_Callback

% --- Executes on slider movement.
function load_slide_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Collect disturbance vector
disturbances = get(handles.load_popup,'UserData');
ind = get(handles.load_popup,'Value');
disturbances(ind) = get(handles.load_slide,'Value');
set(handles.load_text,'String',num2str(disturbances(get(handles.load_popup,'Value'))));
set(handles.load_popup,'UserData',disturbances);
%end load_slide_Callback

% --- Executes on selection change in load_slide.
function load_popup_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Collect disturbance vector
global colour;
disturbances = get(handles.load_popup,'UserData');
%Update Slider Position
set(handles.load_slide,'Value',disturbances(get(hObject,'Value')));
%Update Load Value String
ind = get(handles.load_popup,'Value');
set(handles.load_text,'String',num2str(disturbances(ind)));
set(handles.load_popup,'ForegroundColor',colour{ind});
%end load_popup_Callback

% --- Executes on button press in load_check.
function load_check_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Collect disturbance vector
disturbances = get(handles.load_popup,'UserData');
if(isempty(disturbances))
    update_console(handles,'No Model Information Loaded Yet');
    set(handles.load_check,'Value',0);
else
    if(get(hObject,'Value'))
        %Enable Load Disturbance Objects & Update Value
        set(handles.load_slide,'Enable','on');
        set(handles.load_popup,'Enable','on');
        set(handles.load_text_top,'Enable','on');
        set(handles.load_text,'Enable','on');
        set(handles.load_text,'String',num2str(disturbances(get(handles.load_popup,'Value'))));
    else
        %Disable Load Disturbance Objects
        set(handles.load_slide,'Enable','off');
        set(handles.load_popup,'Enable','off');
        set(handles.load_text_top,'Enable','off');
        set(handles.load_text,'Enable','off');
    end
end
%end load_check_Callback

% --- Executes on slider movement.
function noise_slide_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Collect noise vector
noises = get(handles.noise_popup,'UserData');
ind = get(handles.noise_popup,'Value');
noises(ind) = 10^get(handles.noise_slide,'Value');
set(handles.noise_text,'String',num2str(noises(get(handles.noise_popup,'Value'))));
set(handles.noise_popup,'UserData',noises);
%end noise_slide_Callback

% --- Executes on selection change in noise_popup.
function noise_popup_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Collect noise vector
global colour;
noises = get(handles.noise_popup,'UserData');
%Update Slider Position
set(handles.noise_slide,'Value',log10(noises(get(hObject,'Value'))));
%Update Load Value String
ind = get(handles.noise_popup,'Value');
set(handles.noise_text,'String',num2str(noises(ind)));
set(handles.noise_popup,'ForegroundColor',colour{ind});
%end noise_popup_Callback

% --- Executes on button press in noise_check.
function noise_check_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Collect User Data
noises = get(handles.noise_popup,'UserData');
if(isempty(noises))
    update_console(handles,'No Model Information Loaded Yet');
    set(handles.noise_check,'Value',0);
else
    if(get(hObject,'Value'))
        %Enable Load Disturbance Objects & Update Value
        set(handles.noise_slide,'Enable','on');
        set(handles.noise_popup,'Enable','on');
        set(handles.noise_text_top,'Enable','on');
        set(handles.noise_text,'Enable','on');
        set(handles.noise_text,'String',num2str(noises(get(handles.noise_popup,'Value'))));
    else
        %Disable Load Disturbance Objects
        set(handles.noise_slide,'Enable','off');
        set(handles.noise_popup,'Enable','off');
        set(handles.noise_text_top,'Enable','off');
        set(handles.noise_text,'Enable','off');
    end
end
%end noise_check_Callback


% --- Executes on slider movement.
function state_est_slide_Callback(hObject, eventdata, handles) %#ok<INUSL>
K = get(handles.state_est_slide,'Value');
set(handles.state_est_text,'String',num2str(K));
set(handles.state_est_text,'UserData',K);
%end state_est_slide_Callback

% --- Executes on button press in state_est_check.
function state_est_check_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
if(get(handles.state_est_check,'Value'))
    %Enable Gain Box
    set(handles.state_est_slide,'Enable','on');
    set(handles.state_est_text,'Enable','on');
else
    %Disable Gain Box
    set(handles.state_est_slide,'Enable','off');
    set(handles.state_est_text,'Enable','off');
end
%end state_est_check_Callback

function sample_edit_Callback(hObject, eventdata, handles) %#ok<INUSL>
Ts = str2double(get(handles.sample_edit,'String'));
if(isnan(Ts)); 
    update_console(handles,'Check Sample Time Value'); 
    return
end
if(~Ts); 
    update_console(handles,'Sample Time Must Not be 0'); 
end
set(handles.sample_edit,'UserData',Ts);
%end sample_edit_Callback


%%--------------- START SIMULATION ---------------%%
%Calls create functions and starts Simulation Timer
function ok = start_sim(handles)
%Clear Screen
clc
ok = 0;
%Clear Messages
update_console(handles,[]);
%Collect Data from GUI
MPC1 = create_MPC(handles);
if(~isa(MPC1,'jMPC'))
    update_console(handles,sprintf('--MPC Setup Error - Please Check Messages Above---'));
    tabselectionfcn(gcf,'Tabs',1);
    return
end
%Save jMPC Object
set(handles.top_panel,'UserData',MPC1);

%Create Graphics
[Sout,Sin] = create_graphics(handles);
if(isempty(Sout) || isempty(Sin))
    update_console(handles,sprintf('--Graphics Setup Error---'));
    return
end  

%Collect Plant
Plant = get(handles.plant_list,'UserData');

%Fill in Setpoints
for i = 1:MPC1.sizes.nq_out
    valstp{i} = sprintf('Output %d',i); %#ok<AGROW>
end
set(handles.setpoint_popup,'String',valstp);
set(handles.setpoint_popup,'Value',1,'ForegroundColor','b');

%Create Timer
if(get(handles.speed_check,'Value'))
    Timer_Period = 6/Sout.Frame_Size;
else
    Timer_Period = 3/Sout.Frame_Size;
end
sim_timer = timer('Period',Timer_Period,'BusyMode','queue','ExecutionMode','fixedDelay','Timerfcn',{@sim_timer_fcn,MPC1,Plant,Sout,Sin,handles},...
                   'Errorfcn',{@sim_timer_err,handles});
               
%Reset Plot Vector               
plotvec.u = [];
plotvec.del_u = [];
plotvec.xm = [];
plotvec.xp = [];
plotvec.ym = [];
plotvec.yp = [];
plotvec.setp = [];        
set(handles.data_log_menu,'UserData',plotvec);   

%Start Timer               
start(sim_timer);

%Update GUI
status = 'Running';
set(handles.start_button,'UserData',status);
set(handles.start_button,'String','Stop');
update_console(handles,status)
set(handles.pause_button,'Enable','on');
%Disable Setup Entry
GUI_update(handles,'off');

%Save Timer
set(handles.pause_button,'UserData',sim_timer);
ok = 1;
%end start_sim

%%--------------- STOP SIMULATION ---------------%%
%Stops the simulation and cleans up the variables
function stop_sim(handles)
%Collect, Stop & Delete Timer
sim_timer = get(handles.pause_button,'UserData');
stop(sim_timer);
delete(sim_timer);
status = 'Stopped';    
set(handles.start_button,'UserData',status);    %Save User Data
set(handles.start_button,'String','Start');
set(handles.pause_button,'Enable','off');
%Disable Noise & Disturbance
set(handles.load_check,'Value',0);
set(handles.load_slide,'Enable','off');
set(handles.load_slide,'Value',0);
set(handles.load_text,'Enable','off');
set(handles.load_text,'String',[]);
set(handles.load_text_top,'Enable','off');
set(handles.load_popup,'Enable','off');
set(handles.noise_check,'Value',0);
set(handles.noise_slide,'Enable','off');
set(handles.noise_slide,'Value',-5);
set(handles.noise_text,'Enable','off');
set(handles.noise_text,'String',[]);
set(handles.noise_text_top,'Enable','off');
set(handles.noise_popup,'Enable','off');
set(handles.in_led,'color',[0.878,0.875,0.89]);
set(handles.out_led,'color',[0.878,0.875,0.89]);
update_console(handles,status);
%Enable Setup Entry
GUI_update(handles,'on');
%end stop_sim

%%--------------- SIM TIMER ERROR ---------------%%
%Executes on Timer Error
function sim_timer_err(obj,event,handles) %#ok<INUSL>
update_console(handles,event.Data.message);
stop_sim(handles); %force GUI stop
%end sim_timer_err

%%--------------- SIM TIMER FUNCTION ---------------%%
%Executes on Timer Interrupt to step the simulation one step
function sim_timer_fcn(obj,event,MPC1,Plant,axes_Sout,axes_Sin,handles) %#ok<INUSL>
status = get(handles.start_button,'UserData');
persistent sModel sPlant u u_all del_xm xm xp yp u_op y_op Sin Sout Ts setpflag;
if(strcmp(status,'Stopped')) %status is updated after the timer is initially started
    %Allocate initial values
    sModel = MPC1.Model;
    sPlant = Plant;
    u = MPC1.initial.u;
    u_all = zeros(MPC1.Nb,MPC1.sizes.n_in);
    u_all(1,:) = MPC1.initial.u';
    del_xm = sModel.x0;
    xp = sPlant.x0;
    xm = zeros(MPC1.sizes.states,1);
    %Initial Plant Output & States
    if(isa(sPlant.C,'function_handle'))
        yp = Plant.C([],Plant.x0,u,Plant.param);
    else
        yp = Plant.C*Plant.x0;
    end
    u_op = MPC1.lin.u_op;
    y_op = MPC1.lin.y_op;
    Sin = axes_Sin;
    Sout = axes_Sout;
    Ts = get(handles.sample_edit,'UserData');
    setpflag = 0;
end

if(strcmp(status,'Running'))
    %Reset LED Flags
    f_in = 0;
    f_out = 0;
    %Collect Up To Date Values
    setpoints = get(handles.setpoint_popup,'UserData');
    disturbances = get(handles.load_popup,'UserData');
    noises = get(handles.noise_popup,'UserData');
    demomode = strcmp(get(handles.demo_mode,'Checked'),'on');
    %Check for Demo Mode
    if(demomode && (setpflag>100)) 
        setpoints = gen_demo_setp(setpoints);
        set(handles.setpoint_popup,'UserData',setpoints);
        setpflag = 0; 
    end
    %State Estimator Update
    del_xm = MPC1.state_est.IKC*del_xm + MPC1.state_est.Kest*(yp-y_op);
    %Calculate Input
    del_u = mpcsolve(MPC1,del_xm,u,setpoints',0,1,tic);
    %Scale & select output
    optdel_u = del_u(1:MPC1.sizes.nm_in);

    %Saturate Rate of Change of Input
    for i = 1:MPC1.sizes.n_in
        if(optdel_u(i) > MPC1.constraints.u(i,3))
            optdel_u(i) = MPC1.constraints.u(i,3);
        elseif(optdel_u(i) < -MPC1.constraints.u(i,3))
            optdel_u(i) = -MPC1.constraints.u(i,3);
        end
    end
    %Sum Input Increment
    u = u + optdel_u; 
    %Saturate Inputs
    for i = 1:MPC1.sizes.n_in
        if(u(i) > MPC1.constraints.u(i,2))
            u(i) = MPC1.constraints.u(i,2);
        elseif(u(i) < MPC1.constraints.u(i,1))
            u(i) = MPC1.constraints.u(i,1);
        end
    end
    
    %Sum All Input Increments
    del_u_rs = reshape(del_u,MPC1.sizes.n_in,MPC1.Nb)';
    triones = tril(ones(MPC1.Nb));
    for i = 1:MPC1.sizes.n_in
        u_all(:,i) = u(i) + triones*del_u_rs(:,i);
    end
    u_all = u_all + repmat(u_op',MPC1.Nb,1);
    %Simulate model
    del_xm = sim(sModel,del_xm,optdel_u);
    %Sum State Increment
    xm = xm + del_xm(1:MPC1.sizes.states);
    
    %Update plant input disturbance
    if(get(handles.load_check,'Value'))
        up = u + disturbances + u_op;
    else
        up = u + u_op;
    end
    %Subject Plant To Input
    [xp,yp] = sim(sPlant,xp,up,Ts);   
    %Update plant output disturbance (measurement noise)
    if(get(handles.noise_check,'Value'))   
        yp = yp + noises.*randn(MPC1.sizes.n_out,1);
    end
    %Check Constraints for LEDS
    for i = 1:MPC1.sizes.n_out
        if(~isinf(MPC1.constraints.y(i,1)) && yp(i) <= MPC1.constraints.y(i,1)); f_out = 1; break; end;
        if(~isinf(MPC1.constraints.y(i,2)) && yp(i) >= MPC1.constraints.y(i,2)); f_out = 1; break; end;
    end
    for i = 1:MPC1.sizes.n_in
        if(~isinf(MPC1.constraints.u(i,1)) && abs(u(i) <= MPC1.constraints.u(i,1)-1e-4)); f_in = 1; break; end;
        if(~isinf(MPC1.constraints.u(i,2)) && abs(u(i) >= MPC1.constraints.u(i,2)-1e-4)); f_in = 1; break; end;
        if(~isinf(MPC1.constraints.u(i,3)) && abs(del_u(i)) >= abs(MPC1.constraints.u(i,3)-1e-4)); f_in = 1; break; end;
    end
    if(f_in)
        set(handles.in_led,'color','r');
    else
        set(handles.in_led,'color','g');
    end
    if(f_out)
        if(get(handles.soft_check,'Value'))
            set(handles.out_led,'color','y'); %soft constrained
        else
            set(handles.out_led,'color','r');
        end
    else
        set(handles.out_led,'color','g');
    end
    %Calculate predicted output
    ypred = MPC1.pred.Phi*del_u + MPC1.pred.F*del_xm + repmat(y_op,MPC1.Np,1);
    %Resize To Matrices for plotting
    ypred = reshape(ypred',MPC1.sizes.n_out,MPC1.Np);
    try %auto resize & stop button can cause problems here
        %Check If Resize Enabled
        if(strcmp(get(handles.auto_scale_menu,'Checked'),'on')); autosize = 1; else autosize = 0; end
        %Draw Outputs
        Sout = plot(Sout,autosize,yp',ypred',(setpoints+y_op(1:MPC1.sizes.nq_out))');
        %Draw Inputs
        Sin = stairs(Sin,autosize,(u+u_op)',u_all);
    catch %#ok<CTCH>
        update_console(handles,sprintf('Plot Error: %s',lasterr)); %#ok<LERR>
        stop_sim(handles); %force GUI stop
    end
    %Log Data if Required
    if(strcmp(get(handles.data_log_menu,'Checked'),'on'))
        plotvec = get(handles.data_log_menu,'UserData');
        plotvec.u = [plotvec.u; up'];
        plotvec.del_u = [plotvec.del_u; optdel_u'];
        plotvec.xm = [plotvec.xm; xm'];
        plotvec.xp = [plotvec.xp; xp'];
        plotvec.ym = [plotvec.ym; del_xm(end-MPC1.sizes.n_out+1:end)'];
        plotvec.yp = [plotvec.yp; yp'];
        plotvec.setp = [plotvec.setp; setpoints'];
        set(handles.data_log_menu,'UserData',plotvec);
    end
    setpflag = setpflag + 1;
end
%end sim_timer_fcn

%%--------------- GENERATE DEMO SETPOINTS ---------------%%
%Generate random setpoints for GUI demonstration
function setpoints = gen_demo_setp(setp)
setpoints = setp;
ind = randi([1 length(setp)]);
setpoints(ind) = 1 + 2*randn();
%end gen_demo_setp


%%--------------- READ WORKSPACE ---------------%%
%Read the base workspace for jSS, jNL and lti objects, populate the lists
function read_workspace(handles)
%Initialise
mvars = [];
pvars = [];
model_val = 1; 
plant_val = 1;
j = 1;
%Collect workspace variables
allvars = evalin('base','who');
%Check class of all variables
for i = 1:length(allvars)
    if(isa(evalin('base',allvars{i}),'jSS') || isa(evalin('base',allvars{i}),'lti')) %check is a jSS object
        mvars{j} = allvars{i};  %#ok<AGROW>
        pvars{j} = allvars{i};  %#ok<AGROW>
        if(strcmp(mvars{j},'Model')) %Save position of model
            model_val = j;
        end
        if(strcmp(pvars{j},'Plant')) %Save position of plant
            plant_val = j;
        end
        j = j + 1;
    elseif(isa(evalin('base',allvars{i}),'jNL'))
        pvars{j} = allvars{i}; %#ok<AGROW>
        if(strcmp(pvars{j},'Plant')) %Save position of plant
            plant_val = j;
        end
        j = j+1;
    end
        
end
%Fill in List of Variables found
set(handles.model_list,'String',mvars);
set(handles.plant_list,'String',pvars);
%Automatically select variables with default names
set(handles.model_list,'Value',model_val);
set(handles.plant_list,'Value',plant_val);
%end read_workspace

%%--------------- LOAD MODEL ---------------%%
%Read the model list box, setup selected object for use in simulation
function load_model(handles)
%Collect Variable Names from List
vars = get(handles.model_list,'String');
if(isempty(vars)); 
    update_console(handles,'No Model Available');
    set(handles.model_text,'String',[]);
    set(handles.model_list,'UserData',[]);
    return; 
end;
%Collect Sample Time
Ts = get(handles.sample_edit,'UserData');
if(isnan(Ts) || ~Ts)
    return
end
%Collect Index of Selected Model
ind = get(handles.model_list,'Value');
%Collect Model from workspace
try
    Model = evalin('base',vars{ind});
catch %#ok<CTCH>
    update_console(handles,'No Model Available');
    set(handles.model_text,'String',[]);
    set(handles.model_list,'UserData',[]);
    set(handles.model_list,'String',[]);
    return; 
end
%Check for Auto Ts Completition (uses model)
if(strcmp(get(handles.auto_ts_menu,'Checked'),'on'))
    winsizes = [0.01 0.1 1 10 100 1000];   
    if(Model.Ts <= 0) %continuous
        [rub,t] = step(Model); %#ok<ASGLU>
        Ts = mean(t(2:end)-t(1:end-1));
        if(Ts < 1e-3)
            Ts = 1e-3;
        end
        Ts = roundn(Ts,-3);
    else
        Ts = Model.Ts;
    end
    T = Ts*100;
    for i = 1:6
        if(T <= winsizes(i))
            break
        end
    end
    set(handles.sample_edit,'String',num2str(Ts));
    set(handles.sample_edit,'UserData',Ts);
    set(handles.frame_popup,'Value',i);
    if(get(handles.model_panel,'UserData')) %building mpc only
        update_console(handles,sprintf('Window Length Set to %3g (s)',winsizes(i)));
        update_console(handles,sprintf('Sampling Period Set to %2.3f (s)',Ts));
    end
end 
%Check for lti models
if(isa(Model,'lti'));
    origTs = Model.Ts;
    if(hasdelay(Model))
        if(~Model.Ts)
            dead_time = max(max(Model.iodelay));
            Model = c2d(Model,Ts);
        else
            dead_time = max(max(Model.iodelay))*origTs;
        end
        sdt = dead_time/Ts;
        update_console(handles,'Model Dead Time Approximated'); %done in jSS below
    else
        dead_time = 0; sdt = 0;
    end
    Model = jSS(Model); %convert to jSS form
    Model.x0 = zeros(size(Model.A,1),1); %create default initial states
else
    dead_time = 0; sdt = 0;
    origTs = Model.Ts;
end
%Evaluate Sizes & display in gui
states = size(Model.B,1); n_in = size(Model.B,2); n_out = size(Model.C,1); 
set(handles.model_text,'String',sprintf('%s \n States:         %d\n Inputs:          %d\n Outputs:       %d\n Ts:                %1.2g(s)\n Dead Time:   %2.2f(s)',vars{ind},states,n_in,n_out,origTs,dead_time));
set(handles.model_text,'UserData',sdt); %save dead time in user data
%Update popup boxes for model size
for i = 1:n_in
    valdis{i} = sprintf('Input %d',i); %#ok<AGROW>
end
for i = 1:n_out
    valstp{i} = sprintf('Output %d',i); %#ok<AGROW>
end
%Update setpoints & disturbance inputs
disturbances = zeros(n_in,1);
noises = 10.^(-5*ones(n_out,1));
set(handles.noise_popup,'String',valstp);
set(handles.load_popup,'String',valdis);
set(handles.noise_popup,'Value',1,'ForegroundColor','b');
set(handles.load_popup,'Value',1,'ForegroundColor','b');
%Save User Data
set(handles.model_list,'UserData',Model);
set(handles.noise_popup,'UserData',noises);
set(handles.load_popup,'UserData',disturbances);
%end load_model

%%--------------- LOAD PLANT ---------------%%
%Read the plant list box, setup selected object for use in simulation
function load_plant(handles)
%Collect Variable Names from List
vars = get(handles.plant_list,'String');
if(isempty(vars)); 
    update_console(handles,'No Plant Available'); 
    set(handles.plant_text,'String',[]);
    set(handles.plant_list,'UserData',[]);
    return; 
end;
%Collect Sample Time
Ts = get(handles.sample_edit,'UserData');
if(isnan(Ts) || (Ts <= 0))
    update_console(handles,'Check Controller Sample Time');
    return
end
%Collect Index of Selected Plant
ind = get(handles.plant_list,'Value');
%Collect Plant from workspace
try
    Plant = evalin('base',vars{ind});
catch %#ok<CTCH>
    update_console(handles,'No Plant Available');
    set(handles.plant_text,'String',[]);
    set(handles.plant_list,'UserData',[]);
    set(handles.plant_list,'String',[]);
    return; 
end 
%Check for lti models
if(isa(Plant,'lti'))
    origTs = Plant.Ts;
    if(hasdelay(Plant))
        if(~Plant.Ts)
            dead_time = max(max(Plant.iodelay));
            Plant = c2d(Plant,Ts);
        else
            dead_time = max(max(Plant.iodelay))*origTs;
        end
        sdt = dead_time/Ts;
        update_console(handles,'Plant Dead Time Approximated'); %done in jSS below
    else
        dead_time = 0; sdt = 0;
        origTs = Plant.Ts;
    end
    Plant = jSS(Plant); %convert to jSS form
    Plant.x0 = zeros(size(Plant.A,1),1); %create default initial states
    %Evaluate Sizes & display in gui
    states = size(Plant.B,1); n_in = size(Plant.B,2); n_out = size(Plant.C,1); 
    set(handles.plant_text,'String',sprintf('%s \n States:         %d\n Inputs:          %d\n Outputs:       %d\n Ts:                %1.2g(s)\n Dead Time:   %2.2f(s)',vars{ind},states,n_in,n_out,origTs,dead_time));
elseif(isa(Plant,'jNL'))
    sdt = 0;
    %Evaluate Sizes & display in gui
    set(handles.plant_text,'String',sprintf('%s \n Non-Linear Model\n Fcn: %s \n States:         %d',vars{ind},func2str(Plant.nl_model),length(Plant.x0)));
else
    dead_time = 0; sdt = 0;
    origTs = Plant.Ts;
    %Evaluate Sizes & display in gui
    states = size(Plant.B,1); n_in = size(Plant.B,2); n_out = size(Plant.C,1); 
    set(handles.plant_text,'String',sprintf('%s \n States:         %d\n Inputs:          %d\n Outputs:       %d\n Ts:                %1.2g(s)\n Dead Time:   %2.2f(s)',vars{ind},states,n_in,n_out,origTs,dead_time));
end
set(handles.plant_text,'UserData',sdt); %save dead time in user data
set(handles.plant_list,'UserData',Plant);
%end load_plant

%%--------------- GUI Update ---------------%%
%Used to enable / disable GUI objects during simulation
function GUI_update(handles,val)
set(handles.weights_table,'Enable',val);
set(handles.constraints_table,'Enable',val);
set(handles.state_est_text_top,'Enable',val);
set(handles.state_est_check,'Enable',val);
set(handles.state_est_slide,'Enable',val);
set(handles.state_est_text,'Enable',val);
set(handles.soft_check,'Enable',val);
set(handles.frame_popup,'Enable',val);
set(handles.prediction_edit,'Enable',val);
set(handles.control_edit,'Enable',val);
set(handles.auto_button,'Enable',val);
set(handles.default_button,'Enable',val);
set(handles.plant_list,'Enable',val);
set(handles.model_list,'Enable',val);
set(handles.refresh_button,'Enable',val);
set(handles.qp_popup,'Enable',val);
set(handles.sample_edit,'Enable',val);
set(handles.setpoints_table,'Enable',val);
set(handles.file_menu,'Enable',val);
set(handles.help_menu,'Enable',val);
set(handles.data_log_menu,'Enable',val);
set(handles.auto_ts_menu,'Enable',val);
set(handles.speed_check,'Enable',val);
%end GUI_update

%%--------------- CREATE MPC ---------------%%
%Read GUI object settings, perform error checking and build an MPC
function MPC1 = create_MPC(handles)
%Reset Ok
set(handles.sim_panel,'UserData',0);
set(handles.model_panel,'UserData',1); %indicate to show messages

%Collect Model & Plant
load_model(handles);
load_plant(handles);
%Collect User Data
Model = get(handles.model_list,'UserData');
Plant = get(handles.plant_list,'UserData');
set(handles.model_panel,'UserData',0); %clear ok
%Bail if no plant or model
if(isempty(Model) || isempty(Plant))
    MPC1 = [];
    update_console(handles,'Check a Plant and Model are Selected');
    return
end

%Check Sizes
if(isa(Plant,'jSS'))
    pn_in = size(Plant.B,2);
    pn_out = size(Plant.C,1);
    mn_in = size(Model.B,2);
    mn_out = size(Model.C,1);
    if(pn_in ~= mn_in)
        MPC1 = [];
        update_console(handles,'Plant & Model Input Sizes are Not Equal');
        return
    end
    if(pn_out ~= mn_out)
        MPC1 = [];
        update_console(handles,'Plant & Model Output Sizes are Not Equal');
        return
    end
end
%Collect Horizon Settings
Np = str2double(get(handles.prediction_edit,'String'));
if(isnan(Np)); update_console(handles,'Check Prediction Horizon Value'); end
if(Np > 50); update_console(handles,'Maximum Prediction Horizon is 50'); end
Nc = str2num(get(handles.control_edit,'String')); %#ok<ST2NM>
if(any(isnan(Nc))); update_console(handles,'Check Control Horizon Value'); end

%Collect Dead Time
model_dt = get(handles.model_text,'UserData');
plant_dt = get(handles.plant_text,'UserData');
%Check Np > DeadTime
dt = (max([plant_dt,model_dt])+1);
if(Np < dt)
    MPC1 = [];
    update_console(handles,'Prediction Horizon Must be Longer Than Dead Time');
    update_console(handles,sprintf('Try Prediction = %2.0f',dt));
    return
end

%Read Constants
sample_edit_Callback([], [], handles);
Ts = get(handles.sample_edit,'UserData');
state_est_slide_Callback([], [], handles);
K = get(handles.state_est_text,'UserData');

%Check Model & Plant are correctly sampled
if(Model.Ts ~= Ts)
    if(~Model.Ts) %continuous
        Model = c2d(Model,Ts);
        update_console(handles,sprintf('Continuous Model Converted to Discrete with Ts %2.2g',Model.Ts));
    else %resample
        try
            Model = d2d(Model,Ts);
        catch %#ok<CTCH>
            update_console(handles,sprintf('Re-Sampling Model Failed: %s',lasterr)); %#ok<LERR>
            update_console(handles,'Please Re-Discretize The Model');
            MPC1 = [];
            return
        end
        update_console(handles,sprintf('Discrete Model Resampled with Ts %2.2g',Model.Ts));
    end
end
if(isa(Plant,'jSS'))
    if(Plant.Ts ~= Ts)
        if(~Plant.Ts) %continuous
            Plant = c2d(Plant,Ts);
            update_console(handles,sprintf('Continuous Plant Converted to Discrete with Ts %2.2g',Plant.Ts));
        else %resample
            try
                Plant = d2d(Plant,Ts);
            catch                             %#ok<CTCH>
                update_console(handles,sprintf('Re-Sampling Plant Failed: %s',lasterr)); %#ok<LERR>
                update_console(handles,'Please Re-Discretize The Plant');
                MPC1 = [];
                return;        
            end
            update_console(handles,sprintf('Discrete Plant Resampled with Ts %2.2g',Plant.Ts));
        end
    end
end

%Collect Sizes
n_in = size(Model.B,2);
n_out = size(Model.C,1);
%Collect Weights
wts = get(handles.weights_table,'Data');
if(iscell(wts)) %manually entered data
    wts = str2double(wts);
end
%Bail if no weights present
if(isempty(wts))
    MPC1 = [];
    update_console(handles,'No Weights Present');
    return
end
if(size(wts,1) < max([n_in n_out]))
    update_console(handles,'Check Correct Number of Weights Entered');
    MPC1 = [];
    return
end
uwt = wts(1:n_in,1);
if(any(isnan(uwt))); update_console(handles,'Check Input Weights for Non-Numerical Values'); end
ywt = wts(1:n_out,2);
if(any(isnan(ywt))); update_console(handles,'Check Output Weights for Non-Numerical Values'); end

%Collect Constraints
cons = get(handles.constraints_table,'Data');
if(iscell(cons)) %manually entered data
    cons = str2double(cons);
end
f = 0;
con.u = cons(1:n_in,1:3);
if(any(isnan(con.u))); update_console(handles,'Check Input Constraints for Non-Numerical Values'); f = 1; end
con.y = cons(1:n_out,4:5);
if(any(isnan(con.y))); update_console(handles,'Check Output Constraints for Non-Numerical Values'); f = 1; end
%Bail if no constraints present
if(f)
    MPC1 = [];
    return
end
for i = 1:n_in %check for negative or 0 rate constraint
    if(~con.u(i,3) || con.u(i,3) < 0)
        con.u(i,3) = Inf;
    end
end

%Soft Constraints
if(get(handles.soft_check,'Value'))
    con.slack = 10*ones(1,n_out); %enable soft constraints via small slack penalty
end

%Collect State Esimation Gain
state_est_slide_Callback(handles.state_est_slide, [], handles); %update value
if(get(handles.state_est_check,'Value'))
    %Create Estimation Gain
    [r,c] = size(Model.A);
    Q = diag(K.*ones(max([r c]),1));
    Q = Q(1:r,1:c);
    Kest = dlqe(Model.A,eye(size(Model.A)),Model.C,Q,eye(size(Model.C,1)));
else
    Kest = [];
end

%Create MPC Controller Based on Settings 
sList = get(handles.qp_popup,'String');
switch(sList{get(handles.qp_popup,'Value')})
	case 'jMPC Mehrotra'
		if(exist('mquad_mehrotra','file')==3)
			solver = 'mquad_mehrotra';
		else
			solver = 'quad_mehrotra';
		end
	case 'jMPC Wright'
		if(exist('mquad_wright','file')==3)
			solver = 'mquad_wright';
		else
			solver = 'quad_wright';
		end
	case 'quadprog'; solver = 'quadprog';
	case 'QPC qpip'; solver = 'qpip';
	case 'QPC qpas'; solver = 'qpas';
	case 'OOQP'; solver = 'ooqp';
	case 'CLP'; solver = 'clp';
end
		
opts = jMPCset('QPSolver',solver);
try
    MPC1 = jMPC(Model,Np,Nc,uwt,ywt,con,Kest,opts);
catch %#ok<CTCH>
    update_console(handles,lasterr); %#ok<LERR>
    MPC1 = [];
    return
end
nq_out = MPC1.sizes.nq_out;

%Collect Setpoints from Initial Table
setpoints = get(handles.setpoints_table,'Data');
if(iscell(setpoints)) %manually entered data
    setpoints = str2double(setpoints(:,1));
end
len = length(setpoints); f = 0;
for i = 1:len
    if(isnan(setpoints(i))); f = 1; setpoints(i) = 0; end;
    if(setpoints(i) > 10); f = 1; setpoints(i) = 10; end;
    if(setpoints(i) < -10); f = 1; setpoints(i) = -10; end;
end
if(len < nq_out) %Too short
    setpoints = [setpoints; zeros(len-nq_out,1)]; f = 1;
else %Chop off end
    setpoints = setpoints(1:nq_out);
end
if(f)
    update_console(handles,'Initial Setpoints Modified');
end
setpoints = setpoints - MPC1.lin.y_op(1:nq_out);
set(handles.setpoint_slide,'Value',setpoints(1));

%Save User Data
set(handles.model_list,'UserData',Model);
set(handles.plant_list,'UserData',Plant);
set(handles.setpoint_popup,'UserData',setpoints);

%Signal Read Ok
set(handles.sim_panel,'UserData',1);
%end create_MPC

%%--------------- WRITE GUI ---------------%%
%Write passed data to the GUI objects
function write_GUI(handles,Np,Nc,uwt,ywt,con,setp,Ts,varargin)
%Assign collected data to GUI objects
set(handles.prediction_edit,'String',num2str(Np));
set(handles.control_edit,'String',num2str(Nc));

%Check weight & constraint lengths
ucon = con.u; ycon = con.y;
ulen = length(uwt); ylen = length(ywt);
if(ulen > ylen)
    ywt = [ywt ; zeros(ulen-ylen,1)];
    ycon = [ycon ; zeros(ulen-ylen,2)];
end
if(ulen < ylen)
    uwt = [uwt ; zeros(ylen-ulen,1)];
    ucon = [ucon ; zeros(ylen-ulen,3)];
end 
setpoints = setp(1,:)';
for i = 1:length(setpoints)
    if(setpoints(i) > 10); setpoints(i) = 10; end;
    if(setpoints(i) < -10); setpoints(i) = -10; end;
end

set(handles.weights_table,'Data',[uwt ywt]);
set(handles.constraints_table,'Data',[ucon ycon]);
set(handles.setpoints_table,'Data',setpoints);
set(handles.setpoint_slide,'Value',setpoints(i));
set(handles.setpoint_popup,'Value',1);
set(handles.sample_edit,'String',num2str(Ts));
%Save User Data
set(handles.setpoint_popup,'UserData',setpoints);
set(handles.sample_edit,'UserData',Ts);
%Reset Popup Values
set(handles.setpoint_popup,'Value',1);
set(handles.setpoint_popup,'ForegroundColor','b');
set(handles.load_popup,'Value',1);
set(handles.setpoint_popup,'ForegroundColor','b');
set(handles.noise_popup,'Value',1);
set(handles.setpoint_popup,'ForegroundColor','b');
%Assign optional variables
if(~isempty(varargin))
    assignin('base','Model',varargin{1});
    if(length(varargin) > 1)
        assignin('base','Plant',varargin{2});
        if(length(varargin) > 2)
            state_est = varargin{3};
            set(handles.state_est_slide,'Value',state_est.K);
            set(handles.state_est_text,'String',num2str(state_est.K));
            set(handles.state_est_check,'Value',state_est.En);
            if(length(varargin) > 3)
                opts = varargin{4};
                if(isfield(opts,'window'))
                    set(handles.frame_popup,'Value',opts.window);
                end
                if(isfield(opts,'qp'))
                    set(handles.qp_popup,'Value',opts.qp);
                end
                if(isfield(opts,'soft'))
                    set(handles.soft_check,'Value',opts.soft);
                end
                if(isfield(opts,'notes'))
                    set(handles.desc_edit,'String',opts.notes);
                end
                if(isfield(opts,'demomode'))
                    set(handles.demo_mode,'Checked','on');
                end
            end
        end
    end
    read_workspace(handles);
    load_model(handles);
    load_plant(handles);
end
%end write_GUI

%%--------------- UPDATE CONSOLE ---------------%%
%Display messages in the console
function update_console(handles,str)
if(isempty(str)) %Clear Display
    ud = [];
else
    str = strcat('- ',str);
    ud = get(handles.console_text,'UserData');
    len = length(ud);

    if(len > 9) %max 10 lines (careful as this doesnt work for multiline strings)
        ud(1:9) = ud(2:10);
        ud{10} = str;
    else
        ud{len+1} = str;
    end
end
set(handles.console_text,'String',ud);
set(handles.console_text,'UserData',ud);
%end update_console

%%--------------- READ USER DATA ---------------%%
%Read data stored in GUI User Data properties to build a GUI object
function [GUI_obj,MPC1] = read_user_data(handles)
%Create an MPC controller (error checking basically)
MPC1 = create_MPC(handles);
%Check Read Successful
if(~get(handles.sim_panel,'UserData'))
    update_console(handles,'Save Error - See Above Messages');
    GUI_obj = [];
    return
end
%Create GUI object to save
Model = get(handles.model_list,'UserData');
Plant = get(handles.plant_list,'UserData');
Ts = get(handles.sample_edit,'UserData');
State_Est.En = get(handles.state_est_check,'Value');
State_Est.K = get(handles.state_est_text,'UserData');
setpoints = get(handles.setpoint_popup,'UserData')';
Options.qp = get(handles.qp_popup,'Value');
Options.soft = get(handles.soft_check,'Value');
Options.window = get(handles.frame_popup,'Value');
Options.notes = get(handles.desc_edit,'String');
GUI_obj = jGUI(Plant,Model,MPC1.weights,MPC1.constraints,MPC1.Np,MPC1.Nc,Ts,State_Est,setpoints,Options);
%end read_user_data

%%--------------- CREATE GRAPHICS ---------------%%
%Setup axes settings, frame sizes and create scroll plot objects
function [Sout,Sin] = create_graphics(handles)
%Collect MPC Object
MPC1 = get(handles.top_panel,'UserData');
Ts = MPC1.Model.Ts; Np = MPC1.Np; Nc = MPC1.Nb; con = MPC1.constraints;
%Setup Frame Size
frame_opts = [0.01 0.1 1 10 100 1000];
ind = get(handles.frame_popup,'Value');
Frame_Size = frame_opts(ind)/Ts;
if(Frame_Size > 100); 
    update_console(handles,sprintf('Maximum of 100 Samples Can be Displayed\n-Resetting Window Length to %3g(s)',100*Ts));
    Frame_Size = 100; 
end
if(Frame_Size < 10);
    update_console(handles,sprintf('Minimum of 10 Samples Can be Displayed\n-Resetting Window Length to %3g(s)',10*Ts));
    Frame_Size = 10; 
end
%Check Frame_Size to Np comparison
if(Frame_Size/Np < 2)
    update_console(handles,sprintf('Due To Scaling, The Prediction Length Must be\n a Maximum of %d, or Increase The Window Length',Frame_Size/2));
    Sout = [];
    Sin = [];
    return
end
%Default Y axes limits
y_lim = [-1 1];
%Setup Axes Handles
axes(handles.outPres); cla; %#ok<MAXES>
set(handles.outPres,'XLim',[0 Ts*Frame_Size],'YLim',y_lim...
    ,'Drawmode','fast','Visible','on','NextPlot','add','Xtick',[]);
axes(handles.outPred); cla; %#ok<MAXES>
set(handles.outPred,'XLim',[0 Ts*Np],'YLim',y_lim,'Drawmode','fast',...
    'Visible','on','NextPlot','add','Xtick',[],'Ytick',[]);
axes(handles.inPres); cla; %#ok<MAXES>
set(handles.inPres,'XLim',[0 Ts*Frame_Size],'YLim',y_lim...
    ,'Drawmode','fast','Visible','on','NextPlot','add','Xtick',[]);
axes(handles.inPred); cla; %#ok<MAXES>
set(handles.inPred,'XLim',[0 Ts*Nc],'YLim',y_lim,'Drawmode','fast',...
    'Visible','on','NextPlot','add','Xtick',[],'Ytick',[]);
%Create Scroll Plot Objects
Sout = ScrollPlot(3,Ts,Frame_Size,Np,0,y_lim,{MPC1.sizes.n_out,MPC1.sizes.nq_out},[handles.outPres,handles.outPred],(con.y+MPC1.lin.y_op*[1 1]));
Sin = ScrollPlot(2,Ts,Frame_Size,Nc,0,y_lim,MPC1.sizes.n_in,[handles.inPres,handles.inPred],(con.u+MPC1.lin.u_op*[1 1 0]));
%end create_graphics

function x = roundn(x,n)
%  Compute the exponential factors for rounding at specified
%  power of 10.  Ensure that n is an integer.
factors  = 10 ^ (fix(-n));

%  Set the significant digits for the input data
x = round(x * factors) / factors;
%end roundn

%%--------------- MENU FUNCTIONS ---------------%%
function save_file_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
[filename,pathname] = uiputfile('*.mat','Save jMPC GUI Object As');
%Create GUI object to save
MPC_data = read_user_data(handles);
if(~isa(MPC_data,'jGUI'))
    return
end
if(~isequal(filename,0))
    save([pathname,filename],'MPC_data');
    update_console(handles,sprintf('Saved to File %s',filename));
end
%end save_file_menu_Callback

function save_workspace_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Create GUI object to save
GUI_obj = read_user_data(handles);
if(~isa(GUI_obj,'jGUI'))
    return
end
%Call Save GUI
curpos = get(gcf,'Position');
ok = save_window([curpos(1)+450 curpos(2)+400],GUI_obj);
if(ok); update_console(handles,'Saved to Workspace'); end;
%end save_workspace_menu_Callback

function save_controller_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
%Create GUI object to save
[GUI_obj,MPC1] = read_user_data(handles);
if(~isa(GUI_obj,'jGUI')) %check operation was successful
    return
end
%Call Save GUI
curpos = get(gcf,'Position');
ok = save_window([curpos(1)+450 curpos(2)+400],MPC1);
if(ok); update_console(handles,'Saved to Workspace'); end;
%end save_controller_menu_Callback

function load_file_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
[filename,pathname] = uigetfile('*.mat','Load jMPC GUI Object');
if(~isequal(filename,0))
    load([pathname,filename]);
    %Write Properties to GUI
    write_GUI(handles,MPC_data.Np,MPC_data.Nc,MPC_data.Weights.uwt,MPC_data.Weights.ywt,...
        MPC_data.Constraints,MPC_data.Setpoints,MPC_data.Ts,MPC_data.Model,MPC_data.Plant,...
        MPC_data.State_Est,MPC_data.Options);
end
%end load_file_menu_Callback

function load_workspace_menu_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
%Open GUI
curpos = get(gcf,'Position');
MPC_data = load_window([curpos(1)+400 curpos(2)+300]);  
if(~isa(MPC_data,'jGUI'))
    return
end
%Write Properties to GUI
write_GUI(handles,MPC_data.Np,MPC_data.Nc,MPC_data.Weights.uwt,MPC_data.Weights.ywt,...
    MPC_data.Constraints,MPC_data.Setpoints,MPC_data.Ts,MPC_data.Model,MPC_data.Plant,...
    MPC_data.State_Est,MPC_data.Options);

update_console(handles,'Loaded From Workspace');
%end load_workspace_menu_Callback

function load_example_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
update_console(handles,[]); %clear
%Check we have control systems toolbox
p = which('tf.m');
if(isempty(p))
    ex = MException('jMPC:ControlToolbox','You must have the MATLAB Control Systems Toolbox to load jMPC GUI Examples');
    throwAsCaller(ex);
end
%Open GUI
curpos = get(gcf,'Position');
MPC_data = example_window([curpos(1)+400 curpos(2)+300]);  
if(~isa(MPC_data,'jGUI'))
    return
end
%Write Properties to GUI
write_GUI(handles,MPC_data.Np,MPC_data.Nc,MPC_data.Weights.uwt,MPC_data.Weights.ywt,...
    MPC_data.Constraints,MPC_data.Setpoints,MPC_data.Ts,MPC_data.Model,MPC_data.Plant,...
    MPC_data.State_Est,MPC_data.Options);

update_console(handles,'Loaded Example');
%end load_example_menu_Callback

function exit_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end
delete(handles.figure1)
%end exit_menu_Callback

function auto_scale_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
if(strcmp(get(handles.auto_scale_menu,'Checked'),'on'))
    set(handles.auto_scale_menu,'Checked','off');
else
    set(handles.auto_scale_menu,'Checked','on');
end
%end auto_scale_menu_Callback

function auto_ts_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
if(strcmp(get(handles.auto_ts_menu,'Checked'),'on'))
    set(handles.auto_ts_menu,'Checked','off');
else
    set(handles.auto_ts_menu,'Checked','on');
end
%end auto_ts_menu_Callback

function data_log_menu_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
if(strcmp(get(handles.data_log_menu,'Checked'),'on'))
    set(handles.data_log_menu,'Checked','off');
else
    set(handles.data_log_menu,'Checked','on');
end
%end data_log_menu_Callback

function demo_mode_Callback(hObject, eventdata, handles) %#ok<DEFNU,INUSL>
if(strcmp(get(handles.demo_mode,'Checked'),'on'))
    set(handles.demo_mode,'Checked','off');
else
    set(handles.demo_mode,'Checked','on');
end
%end demo_mode_Callback

function doc_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
web('jmpc_main.html', '-helpbrowser');
%end doc_menu_Callback

function about_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
global guiver;
%Open GUI
curpos = get(gcf,'Position');
about_window([curpos(1)+470 curpos(2)+300],guiver); 
%end about_menu_Callback

%%--------------- WHITE BACKGROUND FUNCTIONS ---------------%%
function prediction_edit_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function status_text_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function control_edit_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function setpoint_slide_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function setpoint_popup_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function dist_slide_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function dist_popup_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function state_est_edit_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function state_est_slide_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function sample_edit_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function qp_popup_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function plant_list_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function model_list_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function noise_slide_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function load_slide_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function noise_popup_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function load_popup_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function desc_edit_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%%--------------- EMPTY FUNCTIONS ---------------%%
function prediction_edit_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function control_edit_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function status_text_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function qp_popup_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function state_est_edit_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function file_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function load_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function edit_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function save_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function saturate_check_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function state_est_text_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function frame_popup_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return
 
function options_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function help_menu_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function desc_edit_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return

function speed_check_Callback(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
return
