function [ distance, x, y ] = LocationOfWeight_circle( x_intersectPlusAndMoving, y_intersectPlusAndMoving, LEN, W, FcPlus, AddtionalWeight )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

global g;

if x_intersectPlusAndMoving>=0&&y_intersectPlusAndMoving>=0
    yaw=-pi+atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
elseif x_intersectPlusAndMoving<0&&y_intersectPlusAndMoving>=0
    yaw=atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
elseif x_intersectPlusAndMoving<=0&&y_intersectPlusAndMoving<0
    yaw=atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
else
    yaw=pi+atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
end
yaw=yaw*180/pi;
distance=0.1*sqrt(x_intersectPlusAndMoving^2+y_intersectPlusAndMoving^2);
dx=-distance*cosd(yaw);
dy=-distance*sind(yaw);
dMx=-dy*FcPlus(3);
dMy=dx*FcPlus(3);
x=dMy/(AddtionalWeight*g);
y=-dMx/(AddtionalWeight*g);

end

