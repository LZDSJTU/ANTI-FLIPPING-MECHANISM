function [Fc, Mc,  x_intersect, y_intersect] = ForceSimplify(M,pm,pb,Fbase2pingtai,Mbase2pingtai,mb,g)
% gΪ��ֵ
% ��ϵ����ƽ̨���Ĵ� 
Madd=cross(pb-pm,Fbase2pingtai)+cross(pb-pm,[0;0;-mb*g]);
Mc=Madd+Mbase2pingtai;
Fc=[0;0;-M*g]+Fbase2pingtai;

% �������ضԱ߽������Ӱ��
dy=Mc(1)/Fc(3);
dx=-Mc(2)/Fc(3);
% x_new_left=-L/2+dx;
% x_new_right=L/2+dx;
% y_new_down=-W/2+dy;
% y_new_up=W/2+dy;
    
% �����������Ľ��㣬�����е�Ч����
x_intersect=(Fc(3)*pm(1)-Fc(1)*pm(3))/Fc(3)-dx;
y_intersect=(Fc(3)*pm(2)-Fc(2)*pm(3))/Fc(3)-dy;
end