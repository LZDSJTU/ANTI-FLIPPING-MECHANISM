function [ distance, x, y ] = LocationOfWeight( x_intersectPlus, y_intersectPlus, LEN, W, FcPlus, AddtionalWeight )
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
% % if x_intersectPlus>=0&&y_intersectPlus>=0
% %     yaw=-atan(y_intersectPlus/x_intersectPlus);
% % elseif x_intersectPlus<0&&y_intersectPlus>=0
% %     yaw=-pi-atan(y_intersectPlus/x_intersectPlus);
% % elseif x_intersectPlus<=0&&y_intersectPlus<0
% %     yaw=pi-atan(y_intersectPlus/x_intersectPlus);
% % else
% %     yaw=-atan(y_intersectPlus/x_intersectPlus);
% % end
% % yaw=yaw*180/pi;

global g;
if x_intersectPlus>LEN/2&&y_intersectPlus>W/2
%     yaw=-atan(y_intersectPlus-W/2)/(x_intersectPlus-LEN/2);
    dx=x_intersectPlus-LEN/2;
    dy=y_intersectPlus-W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus>LEN/2&&y_intersectPlus<=W/2&&y_intersectPlus>=-W/2
%     yaw=-pi/2;
    dx=x_intersectPlus-LEN/2;
    dy=0;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus>LEN/2&&y_intersectPlus<-W/2
    dx=x_intersectPlus-LEN/2;
    dy=y_intersectPlus+W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<=LEN/2&&x_intersectPlus>=-LEN/2&&y_intersectPlus>W/2
    dx=0;
    dy=y_intersectPlus-W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<=LEN/2&&x_intersectPlus>=-LEN/2&&y_intersectPlus<=W/2&&y_intersectPlus>=-W/2
    dx=0;
    dy=0;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<=LEN/2&&x_intersectPlus>=-LEN/2&&y_intersectPlus<-W/2
    dx=0;
    dy=y_intersectPlus+W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<-LEN/2&&y_intersectPlus>W/2
    dx=x_intersectPlus+LEN/2;
    dy=y_intersectPlus-W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<-LEN/2&&y_intersectPlus<=W/2&&y_intersectPlus>=-W/2
    dx=x_intersectPlus+LEN/2;
    dy=0;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
elseif x_intersectPlus<-LEN/2&&y_intersectPlus<-W/2
    dx=x_intersectPlus+LEN/2;
    dy=y_intersectPlus+W/2;
    dMx=-dy*FcPlus(3);
    dMy=dx*FcPlus(3);
    x=dMy/(AddtionalWeight*g);
    y=-dMx/(AddtionalWeight*g);
    
end
% yaw=yaw*180/pi;
distance=sqrt(x^2+y^2);

end

