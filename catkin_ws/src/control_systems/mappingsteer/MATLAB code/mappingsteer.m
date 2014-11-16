function [ w_s_a] = mappingsteer( v_body, w_body )
% This function maps joystick input to steering angle of 6 wheels

% amrv: middle wheel rotation velocity.
% afsa: actural front wheel steering angle.
% rho:  radius of the rover around ICR
% sfsa: starboard front wheel steering angle
% pfsa: port front wheel steering angle
% pmsa: starboard middle wheel steering angle 
% smsa: port middle wheel steering angle
% srsa: starboard front wheel steering angle
% prsa: port front wheel steering angle
% w_s_a: = [pfsa,sfsa,pmsa,smsa,prsa,srsa,afsa]; 

global D B W R

    rho = D.*cot(afsa); % radius of the rover around ICR

amrv = (1/R).*v_body; % middle wheel rotation velocity.
afsa = D.*atan(w_body./amrv); % actural front wheel steering angle.


sfsa = atan(D./(rho+B)); % starboard front wheel steering angle
pfsa = atan(D./(rho-B)); % port front wheel steering angle
pmsa = zeros(size(sfsa));
smsa = zeros(size(sfsa));

srsa = -atan(D./(rho+B)); % starboard front wheel steering angle
prsa = -atan(D./(rho-B)); % port front wheel steering angle
w_s_a = [pfsa,sfsa,pmsa,smsa,prsa,srsa,afsa];

end

