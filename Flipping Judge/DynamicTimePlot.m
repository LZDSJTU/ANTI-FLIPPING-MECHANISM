clear
clc

tic
% -------------------------------规定固定参数---------------------------
g=9.81;
% D-H参数向量，建议计算时以实际数值代入
% 下面一句的d(1)是包含了base在内的尺寸，这里计算力，不能加上base
% d=[0.29; 0.0; 0.0; 0.302; 0; 0.072];
% d=[0.124; 0.0; 0.00000001; 0.302; 0; 0];
d=[0.124; 0.0; 0.0; 0.302; 0; 0.072];
a=[0.0; 0.27; 0.07; 0.0; 0.0; 0.0;];
alpha=[pi/2; 0; pi/2; pi/2; -pi/2; 0];
% 机械臂质量向量,建议计算时以具体数值代入，单位kg
density=1.3871516;
m=[3.067; 3.909; 2.944; 1.328; 0.547; 0.014]*density;

% 机械臂转动惯量，建议计算时以具体数值代入
I1=[14217713.24,0,0;0,10453355.13,0;0,0,14404336.47];
I2=[25955413.92,0,0;0,60312778.06,0;0,0,41570091.24];
I3=[12698239.12,0,1429158.47;0,16712606.42,0;1429158.47,0,8355860.66];
I4=[5253537.57,0,0;0,2846626.17,0;0,0,4013486.27];
I5=[815473.61,0,0;0,892831.51,0;0,0,404892.09];
I6=[1694.27,0,0;0,1657.70,0;0,0,2976.81];
I(1:3,1:3,1)=I1*1e-9;
I(1:3,1:3,2)=I2*1e-9;
I(1:3,1:3,3)=I3*1e-9;
I(1:3,1:3,4)=I4*1e-9;
I(1:3,1:3,5)=I5*1e-9;
I(1:3,1:3,6)=I6*1e-9;
% 规定坐标系i的原点到质心Ci的向量,这里的c1至c6很有可能为负值，其中c5为0
rc=[0, -168.76, -12.09, 0, 0, 0 ; -51.59, 0, 0, -77.3, 0, 0 ; 0, 0, 22.81, 0, -1.1, -7.06]*1e-3;
% 规定坐标系i的原点到坐标系i-1的向量建议计算时以实际数值代入
r=[0, a(2), a(3), 0, 0, 0 ; d(1), 0, 0, d(4), 0, 0 ; 0, 0, 0, 0, 0, d(6)];
% 定义连杆数目
n=6;
% 建立IRB120的D-H参数矩阵
L(1)=Link('d',d(1),'a',a(1),'alpha',alpha(1),'m',m(1),'r',rc(:,1),'I',I(:,:,1));
L(2)=Link('d',d(2),'a',a(2),'alpha',alpha(2),'m',m(2),'r',rc(:,2),'I',I(:,:,2));
L(3)=Link('d',d(3),'a',a(3),'alpha',alpha(3),'m',m(3),'r',rc(:,3),'I',I(:,:,3));
L(4)=Link('d',d(4),'a',a(4),'alpha',alpha(4),'m',m(4),'r',rc(:,4),'I',I(:,:,4));
L(5)=Link('d',d(5),'a',a(5),'alpha',alpha(5),'m',m(5),'r',rc(:,5),'I',I(:,:,5));
L(6)=Link('d',d(6),'a',a(6),'alpha',alpha(6),'m',m(6),'r',rc(:,6),'I',I(:,:,6));
% 建立机器人
IRB120=SerialLink(L,'name','IRB120','tool',[eye(3),[0;0;0.072];0,0,0,1]);

% -------------------------------规定固定参数完---------------------------

% -------------------------------规定初始输入参数---------------------------
% 规定基座线速度、角速度、线加速度、角加速度，建议计算时以实际数值代入，都为0
w0=[0; 0; 0];
w0d=[0; 0; 0];
p0dd=[-5; 0; g];
% 规定末端受到的力和力矩,都为0
f_end=[0; 0; 0];
u_end=[0; 0; 0];
%规定机械臂各关节的角度
q=[0,pi/4,pi/4,0,0,0];
%规定机械臂各关节的角速度
qd=[250*pi/180,-250*pi/180,250*pi/180,320*pi/180,0,0];
%规定机械臂各关节的角加速度
qdd=[100*pi/180,150*pi/180,150*pi/180,0,0,0];

% 规定机械臂在移动平台上的安装位置
pb=[0; 0; 0];
% 定义移动平台的尺寸和重量，长度用LEN原因：L为机械臂连杆参数，重叠，故用LEN
LEN=0.3;
W=0.3;
H=0.2;
M=30;
% 规定移动平台重心位置
pm=[0; 0; -H/2];
% 定义基座base的质量
density=1.3871516;
mb=6.21502*density;
% 定义基座质心相对于底部圆心的向量
rb=[-42.04; 0.08; 79.64]*1e-3;

t=1;
dt=0.01;
period=0.1;
for j=0:dt:period
% -------------------------------规定初始输入参数完---------------------------

% 计算每个关节对应的齐次变换矩阵以及相邻两个坐标系之间的齐次变换矩阵
T(1:4,1:4,1)=L(1).A(q(1));
R(1:3,1:3,1)=T(1:3,1:3,1);
for i=2:n
    RMiddleValue=L(i).A(q(i));
    R(1:3,1:3,i)=RMiddleValue(1:3,1:3);
    T(:,:,i)=T(:,:,i-1)*RMiddleValue;
end

% 递推计算每个连杆的角速度和角加速度、线加速度、质心线加速度
z0=[0; 0; 1];
w(1:3,1)=R(:,:,1)'*(w0+qd(1)*z0);
wd(1:3,1)=R(:,:,1)'*(w0d+qdd(1)*z0+qd(1)*cross(w0,z0));
pdd(1:3,1)=R(:,:,1)'*p0dd+cross(wd(1:3,1),r(1:3,1))+cross(w(1:3,1),cross(w(1:3,1),r(1:3,1)));
pcdd(1:3,1)=pdd(1:3,1)+cross(wd(1:3,1),rc(1:3,1))+cross(w(1:3,1),cross(w(1:3,1),rc(1:3,1)));
for i=2:n
    w(1:3,i)=R(:,:,i)'*(w(1:3,i-1)+qd(i)*z0);
    wd(1:3,i)=R(:,:,i)'*(wd(1:3,i-1)+qdd(i)*z0+qd(i)*cross(w(1:3,i-1),z0));
    pdd(1:3,i)=R(:,:,i)'*pdd(1:3,i-1)+cross(wd(1:3,i),r(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),r(1:3,i)));
    pcdd(1:3,i)=pdd(1:3,i)+cross(wd(1:3,i),rc(1:3,i))+cross(w(1:3,i),cross(w(1:3,i),rc(1:3,i)));
end

% 逆向递推计算关节所受的力和力矩
f(1:3,n)=eye(3)*f_end+m(n)*pcdd(1:3,n);
u(1:3,n)=-cross(f(1:3,n),r(1:3,n)+rc(1:3,n))+eye(3)*u_end+cross(eye(3)*f_end,rc(1:3,n))+...
I(1:3,1:3,n)*wd(1:3,n)+cross(w(1:3,n),I(1:3,1:3,n)*w(1:3,n));
for i=(n-1):-1:1
    f(1:3,i)=R(:,:,i+1)*f(1:3,i+1)+m(i)*pcdd(1:3,i);
    u(1:3,i)=-cross(f(1:3,i),r(1:3,i)+rc(1:3,i))+R(:,:,i+1)*u(1:3,i+1)+cross(R(:,:,i+1)*f(1:3,i+1),rc(1:3,i))+...
    I(1:3,1:3,i)*wd(1:3,i)+ cross(w(1:3,i),I(1:3,1:3,i)*w(1:3,i));
end

% 对平台的反作用力需要加一个负号
Fjoint1=-R(:,:,1)*f(1:3,1);
Mjoint1=-R(:,:,1)*u(1:3,1);
% [0;0;0.166]是因为Fb的作用点不在平台，而在基座的上顶面，即电机1处
Madd=cross([0;0;0.166],Fjoint1)+cross(rb,[0;0;-mb*g]);
Mbase2pingtai=Madd+Mjoint1;
Fbase2pingtai=[0;0;-mb*g]+Fjoint1;

[Fc, Mc, x_intersect, y_intersect]... 
    = ForceSimplifyFun(M,pm,pb,Fbase2pingtai,Mbase2pingtai,mb,g);

Fbase(t,1:3)=Fbase2pingtai;
Mbase(t,1:3)=Mbase2pingtai;
Fcenter(t,1:3)=Fc;
Mcenter(t,1:3)=Mc;
intersect(t,1:2)=[x_intersect; y_intersect];

% figure(1);
% plot([x_new_left,x_new_right,x_new_right,x_new_left,x_new_left],[y_new_down,y_new_down,y_new_up,y_new_up,y_new_down]);
% axis([x_new_left-1 x_new_right+1 y_new_down-1 y_new_up+1]);
% hold on
% axis square
% plot(x_intersect, y_intersect,'*');
% if x_intersect>x_new_right||x_intersect<x_new_left||y_intersect>y_new_up||y_intersect<y_new_down
%     display('倾覆');
% else
%     display('不倾覆');
%     d1=min(x_intersect-x_new_left,x_new_right-x_intersect);
%     d2=min(y_intersect+y_new_down,y_new_up-y_intersect);
%     dmin(t)=min(d1,d2);
% end
% pause

qd=qd+qdd*dt;
q=q+qd*dt;
qplot(t,1:6)=q;
t=t+1;

end


j=0:dt:period;
figure(2);
s=[-1.2 1.2 -0.8 0.8 -0.5 1.2];
IRB120.plot(qplot,'workspace',s,'fps',1/dt)
% 
% figure(2);
% plot(j,intersect(:,1));
% hold on
% plot(j,intersect(:,2))


% figure(3);
% plot(j,-Fbase(:,1));
% figure(4);
% plot(j,-Mbase(:,1));


% axis([-2*LEN 2*LEN -2*W 2*W]);
% axis square



% figure(2);
% plot([-LEN/2,LEN/2,LEN/2,-LEN/2,-LEN/2],[-W/2,-W/2,W/2,W/2,-W/2]);
% axis([-2*LEN 2*LEN -2*W 2*W]);
% axis square
% hold on;
