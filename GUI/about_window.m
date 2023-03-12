function varargout = about_window(varargin)
%Create a GUI for displaing about information

% Jonathan Currie (C)
% Control Engineering 2009

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @about_window_OpeningFcn, ...
                   'gui_OutputFcn',  @about_window_OutputFcn, ...
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


% --- Executes just before about_window is made visible.
function about_window_OpeningFcn(hObject, eventdata, handles, varargin) %#ok<INUSL>
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
%My Setup
set(handles.figure1,'Name','','Units','Pixels');
curpos = get(gcf,'Position');
curpos(1:2) = varargin{1};  
set(handles.figure1,'Position',curpos);
set(handles.version_text,'String',sprintf('Version %1.1f',varargin{2}));


% --- Outputs from this function are returned to the command line.
function varargout = about_window_OutputFcn(hObject, eventdata, handles)  %#ok<INUSL>
varargout{1} = handles.output;
