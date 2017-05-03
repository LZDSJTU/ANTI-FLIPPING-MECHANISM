function [x_intersect, y_intersect,Fc,Mc,dx] = ForceSimplify(M,H,h,pm,pb,Fbase2platform,Mbase2platform,mb,p0dd)
% g>0
% Forces are reduced to center of platform 
global g;
Madd = cross(pb-pm,Fbase2platform)+cross(pb-pm,[0;0;-mb*g]); %??????mbg??
Mc = Madd+Mbase2platform; %??????Madd??
Fc = [0;0;-M*g]+Fbase2platform-M*(p0dd-[0;0;g]);

% Equivalent Displacement
dy = -Mc(1)/Fc(3);
dx = Mc(2)/Fc(3);

% Equivalent Intersection Point
x_intersect = (Fc(3)*pm(1)-Fc(1)*(pm(3)+H+h))/Fc(3)-dx;
y_intersect = (Fc(3)*pm(2)-Fc(2)*(pm(3)+H+h))/Fc(3)-dy;
end