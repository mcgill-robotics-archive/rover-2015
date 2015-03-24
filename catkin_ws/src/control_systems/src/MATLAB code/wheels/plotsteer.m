function plotsteer( w_s_a)

% this code plot rover steering (2D top view), "figure" command are
% outside this function.

% sfsa: starboard front  steering angle
% pfsa: port      front  steering angle
% smsa: starboard middle steering angle
% pmsa: port      middle steering angle
% srsa: starboard rear   steering angle
% prsa: port      rear   steering angle

% w_s_a = [pfsa,sfsa,pmsa,smsa,prsa,srsa,afsa]; 

global D B W R

pfsa = w_s_a(1);
sfsa = w_s_a(2);
pmsa = w_s_a(3);
smsa = w_s_a(4);
prsa = w_s_a(5);
srsa = w_s_a(6);

% plot body and body frame
pos_body = [0 0];
size_body = [2*B,2*D];
size_body_frame = [2*B,2*D];
%function: [handle] = draw2DBox(pos, size, rotation, EdgeColor,LineStyle)
draw2DBox(pos_body, size_body, 0, 'k','-');
%fuction: [handle] = draw2DFrame(pos,size,rotation,EdgeColor,LineStyle)
draw2DFrame([0 0],size_body_frame,0,'k','-.');

% plot wheels
pos_pf = [-B,+D]; %port front
pos_pm = [-B,0]; %port middle
pos_pr = [-B,-D];%port rear
pos_sf = [+B,+D];%starboard front
pos_sm = [+B,0]; %starboard middle
pos_sr = [+B,-D];%starboard rear
size_wheel = [W,2*R]; 

if sfsa<0
    size_fw_frame = [6,R]; % size front wheel frame
    size_mw_frame = [6,R]; % size middle wheel frame
    size_rw_frame = [6,R]; % size rear wheel frame
else 
    size_fw_frame = [-6,R]; % size front wheel frame
    size_mw_frame = [-6,R]; % size middle wheel frame
    size_rw_frame = [-6,R]; % size rear wheel frame
end
draw2DBox(pos_pf, size_wheel, pfsa,'k','-');
draw2DBox(pos_sf, size_wheel, sfsa,'k','-');
draw2DBox(pos_pm, size_wheel, pmsa,'k','-');
draw2DBox(pos_sm, size_wheel, smsa,'k','-');
draw2DBox(pos_pr, size_wheel, prsa,'k','-');
draw2DBox(pos_sr, size_wheel, srsa,'k','-');

% [handle] = draw2DFrame(pos,size,rotation,EdgeColor,LineStyle)
draw2DFrame(pos_pf, size_fw_frame, pfsa,'r','--');
draw2DFrame(pos_sf, size_fw_frame, sfsa,'r','--');
draw2DFrame(pos_pm, size_mw_frame, pmsa,'g','--');
draw2DFrame(pos_sm, size_mw_frame, smsa,'g','--');
draw2DFrame(pos_pr, size_rw_frame, prsa,'b','--');
draw2DFrame(pos_sr, size_rw_frame, srsa,'b','--');

end