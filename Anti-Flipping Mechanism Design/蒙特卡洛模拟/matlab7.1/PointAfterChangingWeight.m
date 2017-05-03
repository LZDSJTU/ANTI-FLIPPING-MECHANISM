function [ x_intersectFinal, y_intersectFinal ] = PointAfterChangingWeight( x, y, FcPlus, McPlus, AddtionalWeight, pm ,H, h )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
global g;
FcFinal=FcPlus;
McFinal=McPlus+cross([x;y;0],[0;0;-AddtionalWeight*g]);
% Equivalent Displacement
dy = -McFinal(1)/FcFinal(3);
dx = McFinal(2)/FcFinal(3);

% Equivalent Intersection Point
x_intersectFinal = (FcFinal(3)*pm(1)-FcFinal(1)*(pm(3)+H+h))/FcFinal(3)-dx;
y_intersectFinal = (FcFinal(3)*pm(2)-FcFinal(2)*(pm(3)+H+h))/FcFinal(3)-dy;

end

