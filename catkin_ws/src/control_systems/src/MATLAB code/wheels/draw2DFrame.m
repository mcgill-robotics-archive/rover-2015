function [handle] = draw2DFrame(pos,size,rotation,EdgeColor,LineStyle)

%DISPLAYFRAME    Displays a frame of reference in space.
%   rotation:   rotation matrix of the frame, in radian
%   position:   [x y] global coordinate of the origin of the frame
%   length:     [length of x axis, length of yaxis]

ro = [pos(1); pos(2);];
% Find rotation matrix of the body
RotM = Rot2DM(rotation);
    AxisX  = size(1)*RotM*[1 0]';
    AxisY  = size(2)*RotM*[0 1]';
    
    
    h1 = line([ro(1),ro(1)+AxisX(1)],...
        [ro(2), ro(2)+AxisX(2)],'Color',EdgeColor,'LineStyle',LineStyle);
    h2 = line([ro(1),ro(1)+AxisY(1)],...
        [pos(2), pos(2)+AxisY(2)],'Color',EdgeColor,'LineStyle',LineStyle);
    
    handle = [h1 h2];

end