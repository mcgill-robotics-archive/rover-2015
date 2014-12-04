% realize 2D kinematic mapping between generalized coordinates p and 
% joint coordinates q. 
clc, clear, close all;

% Define rover configuration parameters, refering figure in the code note. 
global D B W R
D = 50e-2;  % distance between wheels of: front and middle/middle and rear [cm]   
B = 40e-2;  % distance between longitudinal axis and port/startboard wheels  [cm]
R = 16.5e-2; % wheel radius [cm]
W = 15e-2; % wheel width [cm]

% ========= simulate the input signals ======================
% suppose the joystick rotate a whole cycle full range from 0 to 360 deg.
a_joy = (-90:90)'; % angle of joystick input, deg, refering figure in code note
v_body_lim = 0.1; % full range of body velocity
w_body_lim = deg2rad(20); % full range of body rotational velocity
v_body = v_body_lim*sin(deg2rad(a_joy)); % m/s
w_body = w_body_lim*sin(deg2rad(a_joy)); % rad/second

% =============== mapping ===================================

% function [ w_s_a, afsa ] = mappingsteer( v_body, w_body )
[ w_s_a ] = mappingsteer( v_body, w_body );

% ========= 2D animation and figures ======================
%{
figure;
for step = 1:length(a_joy);
clf
subplot(1,2,1)
hold on
axis equal
axis([-2 2 -2 2])
%function: plotsteer( pfsa,sfsa,pmsa,smsa,prsa,srsa)
plotsteer( w_s_a (step,:))
xlabel('lateral axis')
ylabel('longitudinal axis')
title('2D top view, rover local coordination')
hold off

subplot(1,2,2)
hold on
axis ([-20 20 -40 40])
plot(rad2deg(w_body),rad2deg(w_s_a (:,7)))
plot([rad2deg(w_body(step)),rad2deg(w_body(step))],[-40,40],'r--')
plot([-20,20],[rad2deg(w_s_a (step,7)),rad2deg(w_s_a (step,7))],'r--')
xlabel('rotational velocity of rover body, deg/s')
ylabel('actual front steering angle, deg')
hold off
pause(0.05)
end 
%}
%hold off











