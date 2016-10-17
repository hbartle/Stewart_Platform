function [ new_angles ] = calc_servo_angles( rod, servo_arm, leg, beta,  rod_attach_P, servo_attach_B   )
%Calculates new servo angles
%  
%   Input arguments:        rod                 Effective rod length
%                                               calculated with calc_rod_length()
%                           servo_arm           Length of servo arm
%                           leg                 Length of physical
%                                               operating arm
%                           beta                Angle of servo arm plane
%                                               relative to the x-axis of base system
%                           rod_attach_P        Matrix with coordinates of rod
%                                               attachment points in platform system
%                           servo_attach_B      Matrix with coordinates of
%                                               servo attachment points in base system
%   Output Arguments:       new_angles          Matrix with new servo
%                                               angles for all 6 servos
%

%% Get coordinates
x_P= rod_attach_P(1,:);
y_P= rod_attach_P(2,:);
z_P= rod_attach_P(3,:);

x_B= servo_attach_B(1,:);
y_B= servo_attach_B(2,:);
z_B= servo_attach_B(3,:);

%% Calculate auxiliary quatities
L= rod.*rod -leg.*leg + servo_arm*servo_arm;

M= 2*servo_arm*(z_P - z_B);

N= 2*servo_arm*(cos(beta).*(x_P - x_B) + sin(beta).*(y_P - y_B));

new_angles= asin(L./sqrt(M.*M + N.*N))- atan2(N,M);


end

