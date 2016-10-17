%   Function name: rotX()
%   
%   Input arguments:    phi         [rad] or [°] rotation angle
%                       type        [-] Type of anlge (Degree/radian),
%                                       default: radian
%
%   Output arguments:   rotX        [-] rotation matrix

function [ rotX ] = rotX( phi, type )
%rotX Calculates rotational matrix for an rotation around the x-axis
%with angle phi
if nargin < 2
    type= 'radian';
end


if(strcmp(type,'degree'))
    phi=phi*pi/180;
end

rotX= [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
end

