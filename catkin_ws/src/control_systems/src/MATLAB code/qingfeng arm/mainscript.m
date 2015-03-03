% given arm length a1, a2, given the position of end effect, xp yp.
% calculate angle theta: t1, t2. in radian, CCW.

clc,clear, close all
% structure:
a1 = 1;     % arm length1
a2 = 1;   % arm length2
%specify xp,
% xp = 0.1;   
% range_radius = (a1+a2);
% range_yp = (range_radius^2 - xp^2)^(1/2);

% specify yp
yp = 1.5;   
range_radius = (a1+a2);
range_xp = (range_radius^2 - yp^2)^(1/2);

figure;
% for yp = [-range_yp:0.1:range_yp,range_yp]
for xp = [0:0.1:range_xp,range_xp]

    clf

L = (xp^2+yp^2)^(1/2);
theta_p = atan(yp/xp);
rad2deg(theta_p)

theta_1 = acos( (L^2+a1^2-a2^2)/(2*a1*L) ) + theta_p;
rad2deg(theta_1)

theta_2 = acos( (xp - a1*cos(theta_1))/a2 ) - theta_1;
rad2deg(theta_2)

xm = a1*cos(theta_1);
ym = a1*sin(theta_1);

hold on
axis equal
axis([-2 2 -2 2])

rectangle('Position',[-range_radius,-range_radius,2*range_radius,2*range_radius],'Curvature',[1,1]); 

plot([0 xm],[0 ym],'b-','LineWidth',2) % arm1
plot([xm xp], [ym yp],'b-','LineWidth',2) % arm 2
plot(xm, ym, 'k.','MarkerSize',6) % point M
plot(xp, yp, 'k.','MarkerSize',6) % point P
grid on
hold off

display('theta1   theta2  a1   a2')
output = [rad2deg(theta_1) rad2deg(theta_2) norm([xm,ym],2) norm([xp-xm,yp-ym],2)] 
pause

end






