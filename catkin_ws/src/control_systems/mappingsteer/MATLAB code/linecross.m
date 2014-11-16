function [ xc, yc, thetac ] = linecross( pfsa,sfsa  )
% calculate the cross point of two lines
% y = k1*x+b1  and y = k2*x+b2
% xc,yc: the cross point
% thetac: steer angle, imagine a wheel in middle of port and starboard
% the steer angle of the imaginary wheel and the cross point, in radian
% pfsa: steer angle of port front
% sfsa: steer angle of starboard front

global L B

x1 = -B/2; 
y1 = L;
x2 = B/2;
y2 = L;


    k1 = tan(pfsa);
    k2 = tan(sfsa);
    b1 = y1 - k1.*x1;
    b2 = y2 - k2.*x2;

    xc = (b2-b1)./(k1-k2);
    yc = k1.*xc+b1;
    thetac = atan((L-yc)./xc);

%     parallel_index = find ( (pfsa+sfsa)<1e-5 | sign(pfsa)+sign(sfsa) == 0);
    parallel_index = find ( (pfsa+sfsa)<1e-5);
    xc(parallel_index) = inf;
    yc(parallel_index) = L;
    thetac(parallel_index) = 0;
        
end

