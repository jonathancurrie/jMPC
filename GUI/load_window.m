function varargout = load_window(varargin)
%Create a GUI for loading from the Workspace

%   Jonathan Currie (C)
%   Control Engineering 2009 

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @load_window_OpeningFcn, ...
                   'gui_OutputFcn',  @load_window_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before load_window is made visible.
function load_window_OpeningFcn(hObject, eventdata, handles, varargin)
guidata(hObject, handles);

%My Setup
set(handles.figure1,'Name','Load From Workspace','Units','Pixels');
curpos = get(gcf,'Position');
curpos(1:2) = varargin{1};  
set(handles.figure1,'Position',curpos);
read_workspace(handles);
mpc_list_Callback(hObject, eventdata, handles);

uiwait(gcf)

% --- Outputs from this function are returned to the command line.
function varargout = load_window_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;
set(handles.mpc_list,'String',[]);
close(gcf);

% --- Executes on button press in ok_button.
function ok_button_Callback(hObject, eventdata, handles) %#ok<*INUSL,*DEFNU>
% Collect Object
handles.output = get(handles.mpc_list,'UserData');
% Update handles structure
guidata(hObject, handles);
uiresume(gcf); %resume the load function

% --- Executes on button press in cancel_button.
function cancel_button_Callback(hObject, eventdata, handles)
handles.output = [];
% Update handles structure
guidata(hObject, handles);
uiresume(gcf);

% --- Executes on selection change in mpc_list.
function mpc_list_Callback(hObject, eventdata, handles)
%Get MPC Selected
vars = get(handles.mpc_list,'String');
if(isempty(vars)); 
    MPC_data = [];
    set(handles.mpc_list,'UserData',MPC_data);
else
    %Collect Index of Selected MPC
    ind = get(handles.mpc_list,'Value');
    %Collect MPC from workspace
    MPC_data = evalin('base',vars{ind});
    set(handles.mpc_list,'UserData',MPC_data);
end
%Display Statistics
if(~isempty(MPC_data))
    if(isfield(MPC_data.Options,'notes'))
        desc = deblank(MPC_data.Options.notes);
        if(isempty(desc))
            set(handles.mpc_text,'String','No Description Available');
        else
            set(handles.mpc_text,'String',MPC_data.Options.notes);
        end
    else
        set(handles.mpc_text,'String','No Description Available');
    end
else
    set(handles.mpc_text,'String','No Data Available');
end
   
% --- Executes during object creation, after setting all properties.
function mpc_list_CreateFcn(hObject, eventdata, handles) %#ok<INUSD>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function read_workspace(handles)
%Initialise
vars = []; 
j = 1;
%Collect workspace variables
allvars = evalin('base','who');
%Check class of all variables
for i = 1:length(allvars)
    if(isa(evalin('base',allvars{i}),'jGUI')) %check is a jMPC GUI object
        vars{j} = allvars{i};  %#ok<AGROW>
        j = j + 1;
    end
end
%Fill in List of Variables found
set(handles.mpc_list,'String',vars);
