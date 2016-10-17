function [ rod_length ] = calc_rod_length( trans, orient, home_pos, rod_attach_P, servo_attach_B )
%Calculates new rod lengths for given translational and rotational vector
% Using inverse kinematics
%   Input Arguments:        trans           Transitional Vector (x,y,z)'
%                           orient          Orientational Vector 
%                                           (phi,theta, psi)'
%                           rod_attach_P    Matrix with coordinates of rod
%                                           attachment points in platform system
%                           servo_attach_B  Matrix with coordinates of
%                                           servo attachment points in base system
%   Output Arguments        rod_length      Vector containing the effective
%                                           rod length of all 6 rods

trans= trans(:);
orient= orient(:);


%% Calculate rotational matrix

T_BP= rotZ(orient(3))*rotY(orient(2))*rotX(orient(1));

%% Calculate effective rod length in Base System for all 6 rods
for i=1:6
    % l_B Vector 
    l_B= trans + home_pos + T_BP*rod_attach_P(:,i) - servo_attach_B(:,i);
    
    rod_length(i)= norm(l_B);
end

%% Plot positions

cla;

for i=1:6
    rod_attach_P(:,i)= trans + home_pos + T_BP*rod_attach_P(:,i);
end
fill3(servo_attach_B(1,:),servo_attach_B(2,:),servo_attach_B(3,:),'-');
hold on
grid on
fill3(rod_attach_P(1,:),rod_attach_P(2,:),rod_attach_P(3,:),'-g');
axis([-15 15 -15 15 0 25]);
rotate3d on;



for i=1:5
    line([servo_attach_B(1,i+1) rod_attach_P(1,i)],... 
         [servo_attach_B(2,i+1) rod_attach_P(2,i)],...
         [servo_attach_B(3,i+1) rod_attach_P(3,i)],...
         'Color','k');
end

line([servo_attach_B(1,1) rod_attach_P(1,6)],... 
     [servo_attach_B(2,1) rod_attach_P(2,6)],...
     [servo_attach_B(3,1) rod_attach_P(3,6)],...
     'Color','k');

end
