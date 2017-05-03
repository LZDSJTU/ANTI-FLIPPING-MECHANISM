function [ FcPlus, McPlus, x_intersectPlus, y_intersectPlus ] = ForceSimplifyPlus( Fc, Mc, AddtionalWeight, pm ,H, h)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
global g;
FcPlus=Fc+[0; 0; -AddtionalWeight*g];
McPlus=Mc;
% Equivalent Displacement
dy = -McPlus(1)/FcPlus(3);
dx = McPlus(2)/FcPlus(3);

% Equivalent Intersection Point
x_intersectPlus = (FcPlus(3)*pm(1)-FcPlus(1)*(pm(3)+H+h))/FcPlus(3)-dx;
y_intersectPlus = (FcPlus(3)*pm(2)-FcPlus(2)*(pm(3)+H+h))/FcPlus(3)-dy;

end

