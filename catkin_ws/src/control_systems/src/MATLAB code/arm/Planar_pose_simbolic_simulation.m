% Simulation of the function "xy2tht1tht2" for displacement estimaiton
clear; clc; close all;

% Input: EE position in a spherical
n = 20;
t = linspace(1,50,n);
a1 = 2; a2 = sqrt(2);  
x_hat =[2+ 0.5*rand(1,n).*sin(t), 2+ 0.5*sin(t)]; y_hat=[2+0.5*rand(1,n).*cos(t), 2+ 0.5*cos(t)];

% %--
% t2=linspace(1,50,n*2);
% x_hat = abs(2*sin(t2)); y_hat=2*cos(t2);

%----
tht1_0 = pi/4;tht2_0=pi/4;
tht1_est = [tht1_0,zeros(1,n)]; tht2_est = [tht2_0,zeros(1,n)];
% tht1_est(1)=tht1_0; tht2_est(1)=tht2_0;
for i=1:n*2
    [tht1_est(i+1),tht2_est(i+1)] = xy2tht1tht2(a1,a2,tht1_est(i),tht2_est(i),x_hat(i),y_hat(i));
end

x_est = a1*cos(tht1_est)+a2*cos(tht1_est+tht2_est);
y_est = a1*sin(tht1_est)+a2*sin(tht1_est+tht2_est);

figure(1)
plot(x_hat,y_hat,'black*', x_est,y_est,'rs')
hold on
O2_est= [a1*cos(tht1_est);a1*sin(tht1_est)];
plot(O2(1,:),O2(2,:),'o')
hold on

for i=1:n*2+1
     line([0;O2_est(1,i)],[0;O2_est(2,i)])
     line([O2_est(1,i);x_est(i)],[O2_est(2,i);y_est(i)])
     hold on
end

axis equal

% Plot error
err_x = x_est(2:n*2+1)-x_hat;
err_y = y_est(2:n*2+1)-y_hat;

figure(2)
plot(err_x,'b.')
hold on
plot(err_y,'r.')


% will later give the work space of the planar manipulator

