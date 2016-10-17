function varargout = android_panel(varargin)
% ANDROID_PANEL MATLAB code for android_panel.fig
%      ANDROID_PANEL, by itself, creates a new ANDROID_PANEL or raises the existing
%      singleton*.
%
%      H = ANDROID_PANEL returns the handle to a new ANDROID_PANEL or the handle to
%      the existing singleton*.
%
%      ANDROID_PANEL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANDROID_PANEL.M with the given input arguments.
%
%      ANDROID_PANEL('Property','Value',...) creates a new ANDROID_PANEL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before android_panel_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to android_panel_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help android_panel

% Last Modified by GUIDE v2.5 05-Mar-2016 21:34:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @android_panel_OpeningFcn, ...
                   'gui_OutputFcn',  @android_panel_OutputFcn, ...
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


% --- Executes just before android_panel is made visible.
function android_panel_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to android_panel (see VARARGIN)

% Choose default command line output for android_panel
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes android_panel wait for user response (see UIRESUME)
% uiwait(handles.figure1);
handles.stop_it=1;

guidata(hObject,handles);


% --- Outputs from this function are returned to the command line.
function varargout = android_panel_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in connector_pushbutton.
function connector_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to connector_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
try
    connector on 12345
    set(handles.connector_pushbutton,'BackgroundColor','green');
catch
    warndlg('Could not start Connector Service, check if already running');
end
guidata(hObject,handles);

% --- Executes on button press in mobiledev_pushbutton.
function mobiledev_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to mobiledev_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Create Android Device
try
    handles.android= mobiledev;
    set(handles.mobiledev_pushbutton,'BackgroundColor','green');
catch
    warndlg('Could not create Mobile device!')
end
guidata(hObject,handles);

% --- Executes on button press in orientation_pushbutton.
function orientation_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to orientation_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.android.OrientationSensorEnabled=1;
set(handles.orientation_pushbutton,'BackgroundColor','green');
guidata(hObject,handles);

% --- Executes on button press in logging_pushbutton.
function logging_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to logging_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Start Logging

if(handles.android.OrientationSensorEnabled==1)
    handles.android.Logging=1;
    set(handles.logging_pushbutton,'BackgroundColor','green');
else
    warndlg('First Enable Orientation Sensor!');
end
guidata(hObject,handles);

% --- Executes on button press in remote_control_pushbutton.
function remote_control_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to remote_control_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
main_panel= getappdata(0,'stewart_platform');

disp(main_panel);
while(handles.stop_it)
    roll= handles.android.Orientation(:,2);
    pitch= handles.android.Orientation(:,3);
    do_the_stewart_the_second(main_panel,[roll*pi/180 pitch*pi/180 0]);
    drawnow;
end




% --- Executes on button press in close_pushbutton.
function close_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to close_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop_it=0;
guidata(hObject,handles);
close(android_panel);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ angles ] = do_the_stewart_the_second(handles,orient)
%Does the stewart: - Calculates the new angle for given geometry and wanted
%                    position
%                  - Sets the servos connnected with the arduino to the
%                    calculated angles
imag_count=0;
angles= calculate_stewart_platform(handles.r_B,...
                                   handles.r_P,...
                                   handles.servo_arm_length,...
                                   handles.rod_length,...
                                   handles.alpha_B*pi/180,...
                                   handles.alpha_P*pi/180,...
                                   handles.trans,... 
                                   orient);

if(imag(angles(1))~=0)
    set(handles.show_angle1,'Background','red');
else
    set(handles.show_angle1,'Background','default');
    imag_count=imag_count+1;
end
if(imag(angles(2))~=0)
    set(handles.show_angle2,'Background','red');
else
    set(handles.show_angle2,'Background','default');
    imag_count=imag_count+1;
end
if(imag(angles(3))~=0)
    set(handles.show_angle3,'Background','red');
else
    set(handles.show_angle3,'Background','default');
    imag_count=imag_count+1;
end
if(imag(angles(4))~=0)
    set(handles.show_angle4,'Background','red');
else
    set(handles.show_angle4,'Background','default');
    imag_count=imag_count+1;
end
if(imag(angles(5))~=0)
    set(handles.show_angle5,'Background','red');
else
    set(handles.show_angle5,'Background','default');
    imag_count=imag_count+1;
end
if(imag(angles(6))~=0)
    set(handles.show_angle6,'Background','red');
else
    set(handles.show_angle6,'Background','default');
    imag_count=imag_count+1;
end

set(handles.show_angle1,'String', num2str(angles(1)*180/pi));
set(handles.show_angle2,'String', num2str(angles(2)*180/pi));
set(handles.show_angle3,'String', num2str(angles(3)*180/pi));
set(handles.show_angle4,'String', num2str(angles(4)*180/pi));
set(handles.show_angle5,'String', num2str(angles(5)*180/pi));
set(handles.show_angle6,'String', num2str(angles(6)*180/pi));

% Set position if arduino is connected and angles are all real

if(handles.arduino_status && imag_count==6)


    angles= angles*180/pi;
    servo_angles= ([90 90 90 90 90 90]+angles)/180;  

    writePosition(handles.s1, 1-servo_angles(1));
    writePosition(handles.s2, servo_angles(2));
    writePosition(handles.s3, 1-servo_angles(3));
    writePosition(handles.s4, servo_angles(4));
    writePosition(handles.s5, 1-servo_angles(5));
    writePosition(handles.s6, servo_angles(6));
end
