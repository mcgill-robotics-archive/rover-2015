function [ Rot2DM ] = Rot2DMat( theta )
% in 2D frame, give rotational angle (positive as counter clockwise),
% calculate the 2X2 ratation matrix

% theta: rotation angle in CCW direction, in radian
% Rot2DM: rotation matrix R, that: positon final = R * position initial
Rot2DM = [cos(theta) -sin(theta);sin(theta) cos(theta)];

end

