function varargout = save_window(varargin)
%Create a GUI for Saving to the Workspace

%   Jonathan Currie (C)
%   Control Engineering 2009 

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @save_window_OpeningFcn, ...
                   'gui_OutputFcn',  @save_window_OutputFcn, ...
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


% --- Executes just before save_window is made visible.
function save_window_OpeningFcn(hObject, eventdata, handles, varargin)
%My Setup
set(gcf,'Name','Save To Workspace','Units','Pixels');
curpos = get(gcf,'Position');
if(~isempty(varargin))
    curpos(1:2) = varargin{1};
end
set(gcf,'Position',curpos);

%Input Data
handles.data = varargin{2};
if(isa(handles.data,'jGUI'))
    set(handles.name_edit,'String','MPC_GUI1');
else if(isa(handles.data,'jMPC'))
        set(handles.name_edit,'String','MPC1');
    end
end
% Update handles structure
guidata(hObject, handles);

uiwait(gcf)


% --- Outputs from this function are returned to the command line.
function varargout = save_window_OutputFcn(hObject, eventdata, handles) 
varargout{1} = get(handles.ok_button,'UserData');
close(gcf);

% --- Executes on button press in ok_button.
function ok_button_Callback(hObject, eventdata, handles) %#ok<*INUSL,*DEFNU>
% Read File Name String
str = get(handles.name_edit,'String');
str = deblank(str);
f = 0;
for i = 1:length(str)
    if(isspace(str(i)))
        f = 1;
    end
end
if(f)
    set(handles.spaces_text,'Visible','on');
else
    assignin('base',str,handles.data);
    set(handles.ok_button,'UserData',1);
    uiresume(gcf); %resume the load function
end

% --- Executes on button press in cancel_button.
function cancel_button_Callback(hObject, eventdata, handles)
set(handles.ok_button,'UserData',0);
uiresume(gcf);

function name_edit_Callback(hObject, eventdata, handles) %#ok<*INUSD>

% --- Executes during object creation, after setting all properties.
function name_edit_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
