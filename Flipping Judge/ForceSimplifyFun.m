function [Fc, Mc,  x_intersect, y_intersect] = ForceSimplify(M,pm,pb,Fbase2pingtai,Mbase2pingtai,mb,g)
% g为正值
% 力系简化至平台质心处 
Madd=cross(pb-pm,Fbase2pingtai)+cross(pb-pm,[0;0;-mb*g]);
Mc=Madd+Mbase2pingtai;
Fc=[0;0;-M*g]+Fbase2pingtai;

% 计算力矩对边界产生的影响
dy=Mc(1)/Fc(3);
dx=-Mc(2)/Fc(3);
% x_new_left=-L/2+dx;
% x_new_right=L/2+dx;
% y_new_down=-W/2+dy;
% y_new_up=W/2+dy;
    
% 计算力与地面的交点，并进行等效处理
x_intersect=(Fc(3)*pm(1)-Fc(1)*pm(3))/Fc(3)-dx;
y_intersect=(Fc(3)*pm(2)-Fc(2)*pm(3))/Fc(3)-dy;
end