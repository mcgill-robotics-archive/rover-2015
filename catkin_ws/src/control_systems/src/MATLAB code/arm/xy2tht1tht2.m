% Function for getting tht1 tht2 from x y
function [tht1,tht2]=xy2tht1tht2(a1,a2,tht1_0,tht2_0,x,y)


c1_1 = (a1^2*x - y*((- a1^2 + 2*a1*a2 - a2^2 + x^2 + y^2)*(a1^2 + 2*a1*a2 + a2^2 - x^2 - y^2))^(1/2) - a2^2*x + x*y^2 + x^3)/(2*a1*(x^2 + y^2));
c1_2 = (y*((- a1^2 + 2*a1*a2 - a2^2 + x^2 + y^2)*(a1^2 + 2*a1*a2 + a2^2 - x^2 - y^2))^(1/2) + a1^2*x - a2^2*x + x*y^2 + x^3)/(2*a1*(x^2 + y^2));
 
s1_1 = (x*((- a1^2 + 2*a1*a2 - a2^2 + x^2 + y^2)*(a1^2 + 2*a1*a2 + a2^2 - x^2 - y^2))^(1/2) + a1^2*y - a2^2*y + x^2*y + y^3)/(2*a1*(x^2 + y^2));
s1_2 = (a1^2*y - x*((- a1^2 + 2*a1*a2 - a2^2 + x^2 + y^2)*(a1^2 + 2*a1*a2 + a2^2 - x^2 - y^2))^(1/2) - a2^2*y + x^2*y + y^3)/(2*a1*(x^2 + y^2));

tht1_1 = atan(s1_1/c1_1);
tht1_2 = atan(s1_2/c1_2);

tht2_1 = atan((y-a1*sin(tht1_1))/(x-a1*cos(tht1_1)))-tht1_1;
tht2_2 = atan((y-a1*sin(tht1_2))/(x-a1*cos(tht1_2)))-tht1_2;

dev_1 = (tht1_1-tht1_0)^2+(tht2_1-tht2_0)^2;
dev_2 = (tht1_2-tht1_0)^2+(tht2_2-tht2_0)^2;

if dev_1 < dev_2
    tht1 = tht1_1;
    tht2 = tht2_1;
else
    tht1 = tht1_2;
    tht2 = tht2_2;
end

% tht1_deg = tht1*180/pi
% tht2_deg = tht2*180/pi