function varargout = example_window(varargin)
% Create a GUI for selecting an Example System

% Jonathan Currie (C)
% Control Engineering 2009

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @example_window_OpeningFcn, ...
                   'gui_OutputFcn',  @example_window_OutputFcn, ...
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


% --- Executes just before example_window is made visible.
function example_window_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>
% Update handles structure
guidata(hObject, handles);

%My Setup
set(handles.figure1,'Name','Example Loader','Units','Pixels');
curpos = get(gcf,'Position');
curpos(1:2) = varargin{1}; %random problem   
set(handles.figure1,'Position',curpos);
set(handles.example_list,'String',{'Rossiter SISO','Rossiter MIMO','Maciejowski Cessna',...
    'Quanser 3DOF Helicopter','Wood Berry','Lund NL Helicopter','DIW NL Pendulum','Lund NL Quad Tank'});
example_list_Callback([],[],handles);
uiwait(gcf)


% --- Outputs from this function are returned to the command line.
function varargout = example_window_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>
varargout{1} = handles.output;
set(handles.example_list,'String',[]);
close(gcf);


% --- Executes on selection change in example_list.
function example_list_Callback(hObject, eventdata, handles) %#ok<INUSL>
ex = gui_examples(get(handles.example_list,'Value'));
set(handles.example_list,'UserData',ex);
set(handles.example_text,'String',ex.Options.notes);

% --- Executes on button press in ok_button.
function ok_button_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
% Collect Object
handles.output = get(handles.example_list,'UserData');
% Update handles structure
guidata(hObject, handles);
uiresume(gcf); %resume the example function

% --- Executes on button press in cancel_button.
function cancel_button_Callback(hObject, eventdata, handles) %#ok<INUSL,DEFNU>
handles.output = [];
% Update handles structure
guidata(hObject, handles);
uiresume(gcf);

% --- Executes during object creation, after setting all properties.
function example_list_CreateFcn(hObject, eventdata, handles) %#ok<INUSD,DEFNU>
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
