function [handle] = draw2DBox(pos, size, rotation, EdgeColor,LineStyle)

%DRAWBOX    Displays a box in space.
%
%[handle] = drawBox(pos, size, offset, rotation, color, wireframe)
%
%   handle:     handle of the box
%   pos:        position of the box center [x, y]
%   size:       dimensions of the box [x, y, z] local axes
%   rotation:   local rotation of the center of the box wrt to body axes,
%   in radians [rotz] - local axes

% Check sizes
assert( length(pos) == 2,  '2 coordinates required to draw box. Provided: %i', length(pos));
assert( length(size) == 2, '2 dimensions required to draw box. Provided: %i', length(size));
assert( length(rotation) == 1, '1 rotations required to draw box. Provided: %i', length(rotation));

    
% Find coordinates of box center
ro = [pos(1); pos(2);];

% dimensions
sx = 0.5*size(1);
sy = 0.5*size(2);

% Find rotation matrix of the body
RotM = Rot2DM(rotation);

% Find positions of vertices (global coordinates)
v1 = ro+RotM*[-sx; +sy];
v2 = ro+RotM*[+sx; +sy];
v3 = ro+RotM*[+sx; -sy];
v4 = ro+RotM*[-sx; -sy];    
vertices = [v1 v2 v3 v4]';
faces = [1 2 3 4];

% Rendering
handle = patch('Faces',faces,'Vertices',vertices,...
    'FaceColor','w','EdgeColor',EdgeColor,'LineStyle',LineStyle);

end