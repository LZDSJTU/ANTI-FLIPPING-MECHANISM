function [ FcPlusAndMoving, McPlusAndMoving, x_intersectPlusAndMoving, y_intersectPlusAndMoving ] = ForceSimplifyPlusAndMoving( Fc, Mc, AddtionalWeight, pm ,H, h, xsum, ysum)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
global g;
FcPlusAndMoving=Fc+[0; 0; -AddtionalWeight*g];
McPlusAndMoving=Mc+cross([xsum; ysum; 0],[0; 0; -AddtionalWeight*g]);
% Equivalent Displacement
dy = -McPlusAndMoving(1)/FcPlusAndMoving(3);
dx = McPlusAndMoving(2)/FcPlusAndMoving(3);

% Equivalent Intersection Point
x_intersectPlusAndMoving = (FcPlusAndMoving(3)*pm(1)-FcPlusAndMoving(1)*(pm(3)+H+h))/FcPlusAndMoving(3)-dx;
y_intersectPlusAndMoving = (FcPlusAndMoving(3)*pm(2)-FcPlusAndMoving(2)*(pm(3)+H+h))/FcPlusAndMoving(3)-dy;

end

