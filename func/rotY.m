%   Function name: rotY()
%   
%   Input arguments:    theta         [rad] or [°] rotation angle
%                       type        [-] Type of anlge (Degree/radian),
%                                       default: radian
%
%   Output arguments:   rotY        [-] rotation matrix

function [ rotY ] = rotY( theta, type )
%rotX Calculates rotational matrix for an rotation around the y-axis
%with angle phi
if nargin < 2
    type= 'radian';
end


if(strcmp(type,'degree'))
    theta=theta*pi/180;
end

rotY= [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
end

