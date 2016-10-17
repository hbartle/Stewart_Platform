function varargout = stewart_platform(varargin)
% STEWART_PLATFORM MATLAB code for stewart_platform.fig
%      STEWART_PLATFORM, by itself, creates a new STEWART_PLATFORM or raises the existing
%      singleton*.
%
%      H = STEWART_PLATFORM returns the handle to a new STEWART_PLATFORM or the handle to
%      the existing singleton*.
%
%      STEWART_PLATFORM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STEWART_PLATFORM.M with the given input arguments.
%
%      STEWART_PLATFORM('Property','Value',...) creates a new STEWART_PLATFORM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before stewart_platform_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to stewart_platform_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help stewart_platform

% Last Modified by GUIDE v2.5 17-Oct-2016 14:49:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @stewart_platform_OpeningFcn, ...
                   'gui_OutputFcn',  @stewart_platform_OutputFcn, ...
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


% --- Executes just before stewart_platform is made visible.
function stewart_platform_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to stewart_platform (see VARARGIN)

% Choose default command line output for stewart_platform
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes stewart_platform wait for user response (see UIRESUME)
% uiwait(handles.stewart_platform_panel);
% Initialise variables
handles.trans=[0 0 0];
handles.orient=[0 0 0 ];
handles.ardu=0;
handles.arduino_status=0;
handles.servos=0;
handles.toggle=0;
do_the_stewart(handles);
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = stewart_platform_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in exit_pushbutton.
function exit_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to exit_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Exit the GUI
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% X-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function x_slider_Callback(hObject, eventdata, handles)
% hObject    handle to x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_x= 4;           %Maximum translation in x direction
min_x= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');

handles.x= slide * (max_x - min_x)  + min_x;
x_pos= num2str(handles.x);
handles.trans(1)= handles.x;

set(handles.show_x,'String',x_pos);
guidata(hObject, handles);

do_the_stewart(handles);

% --- Executes during object creation, after setting all properties.
function x_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_x= 4;           %Maximum translation in x direction
min_x= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');
handles.x= slide * (max_x - min_x)  + min_x;
handles.trans(1)= handles.x;
guidata(hObject, handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Y-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function y_slider_Callback(hObject, eventdata, handles)
% hObject    handle to y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_y= 4;           %Maximum translation in x direction
min_y= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');

handles.y= slide * (max_y - min_y)  + min_y;
y_pos= num2str(handles.y);
handles.trans(2)= handles.y;

set(handles.show_y,'String',y_pos);
guidata(hObject, handles); 

do_the_stewart(handles);
                                       
% --- Executes during object creation, after setting all properties.
function y_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_y= 4;           %Maximum translation in x direction
min_y= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');
handles.y= slide * (max_y - min_y)  + min_y;
handles.trans(2)= handles.y;
guidata(hObject, handles); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Z-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function z_slider_Callback(hObject, eventdata, handles)
% hObject    handle to z_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_z= 4;           %Maximum translation in x direction
min_z= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');

handles.z= slide * (max_z - min_z)  + min_z;
z_pos= num2str(handles.z);
handles.trans(3)= handles.z;

set(handles.show_z,'String',z_pos);
guidata(hObject, handles);

do_the_stewart(handles);


% --- Executes during object creation, after setting all properties.
function z_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_z= 4;           %Maximum translation in x direction
min_z= -4;          %Minimum translation in x direction

slide= get(hObject,'Value');
handles.z= slide * (max_z - min_z)  + min_z;
handles.trans(3)= handles.z;
guidata(hObject, handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Roll-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function roll_slider_Callback(hObject, eventdata, handles)
% hObject    handle to roll_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_roll= 15*pi/180;        %Maximum roll angle
min_roll= -15*pi/180;          %Minimum roll angle

slide= get(hObject,'Value');

handles.roll= slide * (max_roll - min_roll)  + min_roll;
roll_pos= num2str(handles.roll*180/pi);
handles.orient(1)= handles.roll;

set(handles.show_roll,'String',roll_pos);
guidata(hObject, handles);

do_the_stewart(handles);                                       


% --- Executes during object creation, after setting all properties.
function roll_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_roll= 15*pi/180;        %Maximum roll angle
min_roll= -15*pi/180;          %Minimum roll angle

slide= get(hObject,'Value');
handles.roll= slide * (max_roll - min_roll)  + min_roll;
handles.orient(1)= handles.roll;
guidata(hObject, handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Pitch-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function pitch_slider_Callback(hObject, eventdata, handles)
% hObject    handle to pitch_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_pitch= 15*pi/180;        %Maximum pitch angle
min_pitch= -15*pi/180;       %Minimum pitch angle

% Get Slider value
slide= get(hObject,'Value');
handles.pitch= slide * (max_pitch - min_pitch)  + min_pitch;
handles.orient(2)= handles.pitch;

% Show slider value
pitch_pos= num2str(handles.pitch*180/pi);
set(handles.show_pitch,'String',pitch_pos);

guidata(hObject, handles);

do_the_stewart(handles);
                                       

% --- Executes during object creation, after setting all properties.
function pitch_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pitch_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_pitch= 15*pi/180;        %Maximum pitch angle
min_pitch= -15*pi/180;       %Minimum pitch angle

% Get Slider value
slide= get(hObject,'Value');
handles.pitch= slide * (max_pitch - min_pitch)  + min_pitch;
handles.orient(2)= handles.pitch;
guidata(hObject, handles);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Yaw-Slider %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on slider movement.
function yaw_slider_Callback(hObject, eventdata, handles)
% hObject    handle to yaw_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
max_yaw= 15*pi/180;        %Maximum yaw angle
min_yaw= -15*pi/180;       %Minimum yaw angle

slide= get(hObject,'Value');

handles.yaw= slide * (max_yaw - min_yaw)  + min_yaw;
yaw_pos= num2str(handles.yaw*180/pi);
handles.orient(3)= handles.yaw;

set(handles.show_yaw,'String',yaw_pos);
guidata(hObject, handles);

do_the_stewart(handles);

% --- Executes during object creation, after setting all properties.
function yaw_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yaw_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
% Initial Value
max_yaw= 15*pi/180;        %Maximum yaw angle
min_yaw= -15*pi/180;       %Minimum yaw angle

slide= get(hObject,'Value');

handles.yaw= slide * (max_yaw - min_yaw)  + min_yaw;
handles.orient(3)= handles.yaw;
guidata(hObject, handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Show handles %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in show_handle_pushbutton.
function show_handle_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to show_handle_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp(handles);

% arduino_panel_handles= getappdata(0,'arduino_panel');
% 
% disp(arduino_panel_handles);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Connect Devices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% --- Executes on button press in connect_arduino_pushbutton.
function connect_arduino_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to connect_arduino_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set main handle struct on root, so the arduino_panel can access it.
setappdata(0,'stewart_platform',handles);
arduino_panel();
% wait for user to establish connection
uiwait;

% get the new arduino object and the servo pins
new_handles= getappdata(0,'stewart_platform');
handles.servos= new_handles.servos;
handles.ardu= new_handles.ardu;
handles.arduino_status=new_handles.arduino_status;

handles.s1= servo(handles.ardu,handles.servos(1),'MinPulseDuration', 0.53e-3, 'MaxPulseDuration', 2.45e-3);
handles.s2= servo(handles.ardu,handles.servos(2),'MinPulseDuration', 0.6e-3, 'MaxPulseDuration', 2.5e-3);
handles.s3= servo(handles.ardu,handles.servos(3),'MinPulseDuration', 0.55e-3, 'MaxPulseDuration', 2.48e-3);
handles.s4= servo(handles.ardu,handles.servos(4),'MinPulseDuration', 0.58e-3, 'MaxPulseDuration', 2.47e-3);
handles.s5= servo(handles.ardu,handles.servos(5),'MinPulseDuration', 0.6e-3, 'MaxPulseDuration', 2.51e-3);
handles.s6= servo(handles.ardu,handles.servos(6),'MinPulseDuration', 0.5e-3, 'MaxPulseDuration', 2.4e-3);

% handles.s1= servo(handles.ardu,handles.servos(1));
% handles.s2= servo(handles.ardu,handles.servos(2));
% handles.s3= servo(handles.ardu,handles.servos(3));
% handles.s4= servo(handles.ardu,handles.servos(4));
% handles.s5= servo(handles.ardu,handles.servos(5));
% handles.s6= servo(handles.ardu,handles.servos(6));

guidata(hObject,handles);





% --- Executes on button press in connect_android_pushbutton.
function connect_android_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to connect_android_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%setappdata(0,'stewart_platform',handles);
%android_panel(handles);

handles.toggle= ~handles.toggle;
%UDPComIn=udp('192.168.0.26','LocalPort',5556);
if handles.toggle==1;
    UDPComIn=udp('192.168.0.26','LocalPort',5556);
    count=0;
    % Data
    accel_data= 0; 
    gyro_data= 0;
    filter_data= 0;
    pitch=0;
    roll=0;
    yaw= 0;
    roll_gyro=0;
    pitch_gyro=0;
    yaw_gyro=0;
    % Bias for Gyro and Accelerometer
    accel_bias= [-0.3640    0.0378    0.0222];
    gyro_bias= [-0.0138    0.0040    0.0200];
    pause(0.1)
    %Reading sensor data continuously
    tic
    while ishandle(handles.stewart_platform_panel)     

        % Open Stream
        fopen(UDPComIn);
        % Scan for new data, formatted in CSV format
        csvdata=fscanf(UDPComIn);
        % Get single float values
        scandata=textscan(csvdata,'%f %f %f %f %f %f %f %f %f %f %f %f %f','Delimiter',',' );
        if cellfun(@isempty,scandata)== [0 0 0 0 0 0 0 0 0 0 0 0 0]
          count= count + 1;
          time(count)=toc;
          % Time since last sample
          if count> 1
            delta_t= time(count)-time(count-1);
          else
            delta_t= time(count);
          end
          % Raw sensor data
          accel_data(count,1:3)= [scandata{3},scandata{4},scandata{5}];
          gyro_data(count,1:3)= [scandata{7}, scandata{8}, scandata{9}];
          mag_data(count,1:3)= [scandata{11}, scandata{12}, scandata{13}];
          if count == 1
              mag_data0= mag_data(count,1:3);
          end

          % Norm acceleration measurments
          accel_data(count,1:3)= accel_data(count,1:3)/norm(accel_data(count,1:3));

          % Subtract bias
          %accel_data(count,1:3)= accel_data(count,1:3)-accel_bias;
          gyro_data(count,1:3)= gyro_data(count,1:3) - gyro_bias;

          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%% Calculate angles from gyroscope %%%%%%%%%%%%%%%%%
          % The received gyro data is always in a sensor fixed CS. To get the
          % euler angles in the inertial CS a transformation is required.
    %       
    %       T_gyro= 1/cos(roll)*[ [0 sin(roll) cos(roll)];...
    %                              [0 cos(roll)*cos(pitch) -sin(roll)*cos(pitch)];...
    %                              [cos(yaw) sin(roll)*sin(pitch) cos(roll)*cos(pitch)]];
    % 
    %       gyro_data_corrected(1:3,count)= T_gyro*gyro_data(count,1:3)';




          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%% Calculate angles from Acceleration %%%%%%%%%%%%%%

          m= 0.99;
          roll_accel= atan2(accel_data(count,1),sqrt(accel_data(count,2)^2 + accel_data(count,3)^2));
          pitch_accel= atan2(accel_data(count,2),sqrt(accel_data(count,1)^2 + accel_data(count,3)^2));
          yaw_mag= atan2(-mag_data(count,2)*cos(roll_accel)+mag_data(count,3)*sin(roll_accel),...
                         mag_data(count,1)*cos(pitch_accel)+mag_data(count,2)*sin(pitch_accel)*sin(roll_accel)+mag_data(count,3)*sin(pitch_accel)*cos(roll_accel)); 
          if count == 1
               yaw_mag0= atan2(-mag_data(count,2)*cos(roll_accel)+mag_data(count,3)*sin(roll_accel),...
                               mag_data(count,1)*cos(pitch_accel)+mag_data(count,2)*sin(pitch_accel)*sin(roll_accel)+mag_data(count,3)*sin(pitch_accel)*cos(roll_accel));
          end
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%% Complementary filter %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          c=0.98;
          k=1;

          roll= c*(roll- k*gyro_data(count,2)*(delta_t)) + (1-c)*(roll_accel);
          pitch= c*(pitch+ k*gyro_data(count,1)*(delta_t)) + (1-c)*(pitch_accel);
          %yaw= c*(yaw + k*gyro_data(count,3)*delta_t) + (1-c)*(-yaw_mag0+yaw_mag);
          
          filter_data(count,1:3)= [-roll, -pitch, yaw];
          handles.orient(1:3)=filter_data(count,1:3);
          do_the_stewart(handles);
        end
        fclose(UDPComIn);
        pause(0.01);
    end
end
fclose(UDPComIn);
guidata(hObject,handles);

          
          
% wait for user to establish connection
uiwait;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% Geometry Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r_B_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to r_B_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r_B_inputbox as text
%        str2double(get(hObject,'String')) returns contents of r_B_inputbox as a double
handles.r_B= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles);                                       

% --- Executes during object creation, after setting all properties.
function r_B_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r_B_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial Value
handles.r_B= str2num(get(hObject,'String'));
guidata(hObject,handles);

function r_P_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to r_P_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r_P_inputbox as text
%        str2double(get(hObject,'String')) returns contents of r_P_inputbox as a double
handles.r_P= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles); 

% --- Executes during object creation, after setting all properties.
function r_P_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r_P_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial Value
handles.r_P= str2num(get(hObject,'String'));
guidata(hObject,handles);

function servo_arm_length_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to servo_arm_length_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of servo_arm_length_inputbox as text
%        str2double(get(hObject,'String')) returns contents of servo_arm_length_inputbox as a double
handles.servo_arm_length= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles);

% --- Executes during object creation, after setting all properties.
function servo_arm_length_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to servo_arm_length_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial Value
handles.servo_arm_length= str2num(get(hObject,'String'));
guidata(hObject,handles);

function rod_length_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to rod_length_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rod_length_inputbox as text
%        str2double(get(hObject,'String')) returns contents of rod_length_inputbox as a double
handles.rod_length= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles);
                                       
                                       
                                       
% --- Executes during object creation, after setting all properties.
function rod_length_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rod_length_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial Value
handles.rod_length= str2num(get(hObject,'String'));
guidata(hObject,handles);

function alpha_B_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to alpha_B_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha_B_inputbox as text
%        str2double(get(hObject,'String')) returns contents of alpha_B_inputbox as a double
handles.alpha_B= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles);
                                       
% --- Executes during object creation, after setting all properties.
function alpha_B_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha_B_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial value
handles.alpha_B= str2num(get(hObject,'String'));
guidata(hObject,handles);

function alpha_P_inputbox_Callback(hObject, eventdata, handles)
% hObject    handle to alpha_P_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of alpha_P_inputbox as text
%        str2double(get(hObject,'String')) returns contents of alpha_P_inputbox as a double
handles.alpha_P= str2num(get(hObject,'String'));
guidata(hObject,handles);

do_the_stewart(handles);



% --- Executes during object creation, after setting all properties.
function alpha_P_inputbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to alpha_P_inputbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% Initial value
handles.alpha_P= str2num(get(hObject,'String'));
guidata(hObject,handles);

% --- Executes on button press in roll_pitch_sweep_pushbutton.
function roll_pitch_sweep_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to roll_pitch_sweep_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.roll_pitch_sweep_pushbutton,'BackgroundColor','green');
for i=0:0.1:2*pi
    handles.orient=[10*pi/180*cos(i) 10*pi/180*sin(i) 0];
    
    do_the_stewart(handles);
    drawnow
end
for i=2*pi:-0.1:1
    handles.orient=[10*pi/180*cos(i) 10*pi/180*sin(i) 0];
    
    do_the_stewart(handles);
    drawnow
end
set(handles.roll_pitch_sweep_pushbutton,'BackgroundColor','default');
handles.orient=[0 0 0];
guidata(hObject,handles);

% --- Executes on button press in trans_sweep_pushbutton.
function trans_sweep_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to trans_sweep_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.trans_sweep_pushbutton,'BackgroundColor','green');
for i=0:0.1:2*pi
    handles.trans=[4*sin(i) 0 0];
    
    do_the_stewart(handles);
    drawnow
end
for i=0:0.1:2*pi
    handles.trans=[0 4*sin(i) 0];
    
    do_the_stewart(handles);
    drawnow
end
for i=0:0.1:2*pi
    handles.trans=[0 0 2*sin(i)];
    
    do_the_stewart(handles);
    drawnow
end
set(handles.trans_sweep_pushbutton,'BackgroundColor','default');
handles.orient=[0 0 0];
guidata(hObject,handles);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ angles ] = do_the_stewart(handles)
%Does the stewart: - Calculates the new angle for given geometry and wanted
%                    position
%                  - Sets the servos connnected with the arduino to the
%                    calculated angles
tic;
imag_count=0;
angles= calculate_stewart_platform(handles.r_B,...
                                   handles.r_P,...
                                   handles.servo_arm_length,...
                                   handles.rod_length,...
                                   handles.alpha_B*pi/180,...
                                   handles.alpha_P*pi/180,...
                                   handles.trans,... 
                                   handles.orient);

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
lag=toc;
disp('Calc duration: ')
disp(lag)
