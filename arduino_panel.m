function varargout = arduino_panel(varargin)
% ARDUINO_PANEL MATLAB code for arduino_panel.fig
%      ARDUINO_PANEL, by itself, creates a new ARDUINO_PANEL or raises the existing
%      singleton*.
%
%      H = ARDUINO_PANEL returns the handle to a new ARDUINO_PANEL or the handle to
%      the existing singleton*.
%
%      ARDUINO_PANEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARDUINO_PANEL.M with the given input arguments.
%
%      ARDUINO_PANEL('Property','Value',...) creates a new ARDUINO_PANEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before arduino_panel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to arduino_panel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help arduino_panel

% Last Modified by GUIDE v2.5 29-Feb-2016 07:46:09

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @arduino_panel_OpeningFcn, ...
                   'gui_OutputFcn',  @arduino_panel_OutputFcn, ...
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


% --- Executes just before arduino_panel is made visible.
function arduino_panel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to arduino_panel (see VARARGIN)

% Choose default command line output for arduino_panel
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes arduino_panel wait for user response (see UIRESUME)
% uiwait(handles.arduino_panel);
handles.arduino_status=0;
%handles.main_handles= varargin{1};

guidata(hObject,handles);

% --- Outputs from this function are returned to the command line.
function varargout = arduino_panel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in connect_arduino_pushbutton.
function connect_arduino_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to connect_arduino_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


main_panel= getappdata(0,'stewart_platform');

disp(main_panel);


port= get(handles.port_inputbox,'String');
board_type= get(handles.board_type_inputbox, 'String');



    try
        main_panel.ardu= arduino(port,board_type);
        main_panel.arduino_status=1;
        set(handles.connect_arduino_pushbutton,'Backgroundcolor','green');
        set(handles.connect_arduino_pushbutton,'String', 'Connected');
        
    catch
        warning('Unable to connect Arduino');
        warndlg('Unable to connect Arduino. Check if you used the correct port!');
        handles.arduino_status=0;
        set(handles.connect_arduino_pushbutton,'Backgroundcolor','white');
        set(handles.connect_arduino_pushbutton,'String', 'Connect Arduino');
    end



servos(1)= str2num(get(handles.servo1_pin_inputbox,'String'));
servos(2)= str2num(get(handles.servo2_pin_inputbox,'String'));
servos(3)= str2num(get(handles.servo3_pin_inputbox,'String'));
servos(4)= str2num(get(handles.servo4_pin_inputbox,'String'));
servos(5)= str2num(get(handles.servo5_pin_inputbox,'String'));
servos(6)= str2num(get(handles.servo6_pin_inputbox,'String'));
main_panel.servos=servos;
setappdata(0,'stewart_platform',main_panel);

disp(main_panel);
guidata(hObject,handles);

uiresume;




function port_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to port_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of port_inputbox as text
%        str2double(get(hObject,'String')) returns contents of port_inputbox as a double


% --- Executes during object creation, after setting all properties.
function port_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to port_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function board_type_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to board_type_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of board_type_inputbox as text
%        str2double(get(hObject,'String')) returns contents of board_type_inputbox as a double


% --- Executes during object creation, after setting all properties.
function board_type_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to board_type_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function servo1_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo1_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo1_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo1_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo1_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo1_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function servo2_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo2_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo2_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo2_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo2_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo2_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function servo3_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo3_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo3_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo3_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo3_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo3_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function servo4_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo4_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo4_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo4_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo4_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo4_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function servo5_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo5_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo5_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo5_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo5_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo5_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function servo6_pin_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo6_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo6_pin_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo6_pin_inputbox as a double


% --- Executes during object creation, after setting all properties.
function servo6_pin_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo6_pin_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in close_pushbutton.
function close_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to close_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(handles.arduino_panel);


% --- Executes on button press in show_handle_pushbutton.
function show_handle_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to show_handle_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp(handles);
