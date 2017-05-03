function [ distance, x, y ] = LocationOfWeight( x_intersectPlusAndMoving, y_intersectPlusAndMoving, LEN, W, FcPlusAndMoving, AddtionalWeight )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% % if x_intersectPlusAndMoving>=0&&y_intersectPlusAndMoving>=0
% %     yaw=-atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
% % elseif x_intersectPlusAndMoving<0&&y_intersectPlusAndMoving>=0
% %     yaw=-pi-atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
% % elseif x_intersectPlusAndMoving<=0&&y_intersectPlusAndMoving<0
% %     yaw=pi-atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
% % else
% %     yaw=-atan(y_intersectPlusAndMoving/x_intersectPlusAndMoving);
% % end
% % yaw=yaw*180/pi;

global g;
if x_intersectPlusAndMoving>LEN/2&&y_intersectPlusAndMoving>W/2
%     yaw=-atan(y_intersectPlusAndMoving-W/2)/(x_intersectPlusAndMoving-LEN/2);
    dx=x_intersectPlusAndMoving-LEN/2;
    dy=y_intersectPlusAndMoving-W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving>LEN/2&&y_intersectPlusAndMoving<=W/2&&y_intersectPlusAndMoving>=-W/2
%     yaw=-pi/2;
    dx=x_intersectPlusAndMoving-LEN/2;
    dy=0;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving>LEN/2&&y_intersectPlusAndMoving<-W/2
    dx=x_intersectPlusAndMoving-LEN/2;
    dy=y_intersectPlusAndMoving+W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<=LEN/2&&x_intersectPlusAndMoving>=-LEN/2&&y_intersectPlusAndMoving>W/2
    dx=0;
    dy=y_intersectPlusAndMoving-W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<=LEN/2&&x_intersectPlusAndMoving>=-LEN/2&&y_intersectPlusAndMoving<=W/2&&y_intersectPlusAndMoving>=-W/2
    dx=0;
    dy=0;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<=LEN/2&&x_intersectPlusAndMoving>=-LEN/2&&y_intersectPlusAndMoving<-W/2
    dx=0;
    dy=y_intersectPlusAndMoving+W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<-LEN/2&&y_intersectPlusAndMoving>W/2
    dx=x_intersectPlusAndMoving+LEN/2;
    dy=y_intersectPlusAndMoving-W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<-LEN/2&&y_intersectPlusAndMoving<=W/2&&y_intersectPlusAndMoving>=-W/2
    dx=x_intersectPlusAndMoving+LEN/2;
    dy=0;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlusAndMoving<-LEN/2&&y_intersectPlusAndMoving<-W/2
    dx=x_intersectPlusAndMoving+LEN/2;
    dy=y_intersectPlusAndMoving+W/2;
    dMx=-dy*FcPlusAndMoving(3);
    dMy=dx*FcPlusAndMoving(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
    
end
% yaw=yaw*180/pi;
distance=sqrt(x^2+y^2);

end

