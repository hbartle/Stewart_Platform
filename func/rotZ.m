%   Function name: rotZ()
%   
%   Input arguments:    psi         [rad] or [°] rotation angle
%                       type        [-] Type of anlge (Degree/radian),
%                                       default: radian
%
%   Output arguments:   rotZ        [-] rotation matrix

function [ rotZ ] = rotZ( psi, type )
%rotX Calculates rotational matrix for an rotation around the x-axis
%with angle phi
if nargin < 2
    type= 'radian';
end

if(strcmp(type,'degree'))
    psi=psi*pi/180;
end

rotZ= [ cos(psi) sin(psi) 0;-sin(psi) cos(psi) 0; 0 0 1];
end

