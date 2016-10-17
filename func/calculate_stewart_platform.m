function [ angles ] = calculate_stewart_platform( r_B, r_P, servo_arm_length, rod_length, alpha_B, alpha_P, trans, orient)
%Calculate the angles for a rotary stewart platform
% This function calculates the coordinates of a stewart platform with given
% geometry. It then uses Inverse kinematics to calculate the needed leg 
% length for the new given position of the platform. After that the needed 
% for six servo motors are calculated. In the end the whole stewart
% platform is plotted in a three dimensional CS.
%
% Throught several input parameter it is possible to define your stewart
% platform and then analyse the range of movement of your design.
%
% Input parameters: 
%       
%       r_B             [m] Radius of the circumcircle of the base hexagon. 
%                           The points where the servo arm is connected to
%                           the servo axis lie on this circle.
%
%       r_P             [m] Radius of the circumcircle of the platform
%                           hexagon. The points where the rod is attached
%                           to the platform lie on this circle.
%       
%       servo_arm_length [m]    Length of the servo arm. All servo arms  
%                           should have the same length.
%
%       rod_length      [m] Length of the rod connecting the servo arm to
%                           the platform.
%       
%       alpha_B         [rad]   Half the angle between two servos arm    
%                           attachment points. The two servo's arms must  
%                           lie in the same plane.
%
%       alpha_P         [rad]   Half the angle between two rod attachment
%                           points. They also have to be on the same edge
%                           of the platform hexagon.
%
%       trans           [m] The vector which describes the wanted 
%                           translational movement of the platform. 
%                           [x, y, z]
%
%       orient          [rad]   The vector which describes the wanted
%                           rotational movement of the platform according
%                           to the three euler angles [ phi, theta, psi].
%                           
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       Author      Hannes Bartle
%                   hannes@michaelbartle.de
%                   Friedrichshafen, February 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

trans= trans(:);
orient= orient(:);
%% Define the Geometry of the platform

% Beta is the angle between the plane in which the servo arm moves and the
% xz-plane of the base CS.
beta= [ pi+pi/2, pi/2,...
        2*pi/3+pi+pi/2, 2*pi/3+pi/2,...
        4*pi/3+pi+pi/2, 4*pi/3+pi/2];

% Theta_B represents the direction of the points where the servo arm is
% attached to the servo axis. We calculate in polar coordinates.
theta_B= [alpha_B,... 
          alpha_B,...
          pi/3+alpha_B,... 
          pi/3-alpha_B,... 
          pi/3-alpha_B,... 
          pi/3+alpha_B];

% Theta_P represents the direction of the points where the rod is
% attached to the platform. We calculate in polar coordinates.
theta_P= [pi/3-alpha_P,... 
          pi/3-alpha_P,...
          pi/3+alpha_P,... 
          alpha_P,... 
          alpha_P,... 
          pi/3+alpha_P];
      
     
% These are the coordinates of the points where servo arms are attached to 
% the corresponding servo axis.

% Format:
%       x1      x2 ...
%       y1      y2 ...
%       z1      z2 ...
%
servo_attach_B= r_B* [ [cos(theta_B(1)), -sin(theta_B(1)), 0]',...
                     [cos(theta_B(2)), sin(theta_B(2)), 0]',...
                     [-cos(theta_B(3)), sin(theta_B(3)), 0]',...
                     [-cos(theta_B(4)), sin(theta_B(4)), 0]',...
                     [-cos(theta_B(5)), -sin(theta_B(5)), 0]',...
                     [-cos(theta_B(6)), -sin(theta_B(6)), 0]'];

%Coordinates of the points where the rod is attached to the platform.
% Format:
%       x1      x2 ...
%       y1      y2 ...
%       z1      z2 ...
%
rod_attach_P= r_P*[ [cos(theta_P(1)), -sin(theta_P(1)), 0]',...
                      [cos(theta_P(2)), sin(theta_P(2)), 0]',...
                      [cos(theta_P(3)), sin(theta_P(3)), 0]',...
                      [-cos(theta_P(4)), sin(theta_P(4)), 0]',...
                      [-cos(theta_P(5)), -sin(theta_P(5)), 0]',...
                      [cos(theta_P(6)), -sin(theta_P(6)), 0]'];
                                   
% Definition of the platform home position. The home position will be the
% position in which the angle between servo arm and rod is 90°. Because of
% the symmetric arrangement of the servos, h0 must be the same for all six
% positions.
h= sqrt(rod_length.^2+ servo_arm_length.^2 -(rod_attach_P(1,:) - servo_attach_B(1,:)).^2 - (rod_attach_P(2,:) - servo_attach_B(2,:)).^2) -rod_attach_P(3,:);
home_pos= [0, 0, h(1)]';


%% Calculate the needed leg length
% Calculate the transformation matrix for a transform from platform to base
% system with the given euler angles. If errors occure, you have to define
% the transformation matrices around x-, y- and z-axis for yourself.
T_BP= rotZ(orient(3))*rotY(orient(2))*rotX(orient(1)); 


% Calculate the leg vector and leg length for the new position of the
% platform for each servo.
for i=1:6
     
    leg(:,i)= trans + home_pos + T_BP*rod_attach_P(:,i) - servo_attach_B(:,i);
    
    leg_length(i)= norm(leg(:,i));
end

%% Calculate the new servo angles
% Get coordinates of the points where the rod is attached to the platform 
x_P= leg(1,:) + servo_attach_B(1,:);
y_P= leg(2,:) + servo_attach_B(2,:);
z_P= leg(3,:) + servo_attach_B(3,:);

% Get coordinates of the points where the servo arm is attached to the
% servo axis.
x_B= servo_attach_B(1,:);
y_B= servo_attach_B(2,:);
z_B= servo_attach_B(3,:);

% Calculate auxiliary quatities L, N and M
L= leg_length.^2 -rod_length.^2 + servo_arm_length.^2;

M= 2*servo_arm_length*(z_P  - z_B);

for i= 1:6
    N= 2*servo_arm_length*(cos(beta(i)).*(x_P(i) - x_B(i)) + sin(beta(i)).*(y_P(i) - y_B(i)));
    
    % The wanted position could be achieved if the solution of this
    % equation is real for all i
    
    angles(i)= asin(L(i)./sqrt(M(i)^2 + N^2))- atan2(N,M(i));
    
    % Get postion of the point where a spherical joint connects servo arm and
    % rod.

    joint_B(:,i)= [ servo_arm_length*cos(angles(i)).*cos(beta(i)) + servo_attach_B(1,i);...
                    servo_arm_length*cos(angles(i)).*sin(beta(i)) + servo_attach_B(2,i);...
                    servo_arm_length*sin(angles(i))];
end



%% Plot the stewart platform

cla;

leg= leg + servo_attach_B;
fill3(servo_attach_B(1,:),servo_attach_B(2,:),servo_attach_B(3,:),'-');
hold on
grid on
fill3(leg(1,:),leg(2,:),leg(3,:),'-g');
axis([ -r_B-servo_arm_length, r_B+servo_arm_length,...
       -r_B-servo_arm_length, r_B+servo_arm_length,...
       -servo_arm_length rod_length+servo_arm_length]);
rotate3d on;



for i=1:6
    line([servo_attach_B(1,i) joint_B(1,i)],... 
         [servo_attach_B(2,i) joint_B(2,i)],...
         [servo_attach_B(3,i) joint_B(3,i)],...
         'Color','r','LineWidth',2);

    line([joint_B(1,i) leg(1,i)],... 
         [joint_B(2,i) leg(2,i)],...
         [joint_B(3,i) leg(3,i)],...
         'Color','k');
     
    line([servo_attach_B(1,i) leg(1,i)],... 
         [servo_attach_B(2,i) leg(2,i)],...
         [servo_attach_B(3,i) leg(3,i)],...
         'Color','y');
end



end

