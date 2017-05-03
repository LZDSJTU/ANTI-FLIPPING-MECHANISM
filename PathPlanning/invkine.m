function q= invkine( FK )
% D-H参数向量，建议计算时以实际数值代入
d=[0.124; 0.0; 0.0; 0.302; 0; 0.072];
a=[0.0; 0.27; 0.07; 0.0; 0.0; 0.0;];
alpha=[pi/2; 0; pi/2; pi/2; -pi/2; 0];
% 建立IRB120的D-H参数矩阵
L(1)=Link('d',d(1),'a',a(1),'alpha',alpha(1));
L(2)=Link('d',d(2),'a',a(2),'alpha',alpha(2));
L(3)=Link('d',d(3),'a',a(3),'alpha',alpha(3));
L(4)=Link('d',d(4),'a',a(4),'alpha',alpha(4));
L(5)=Link('d',d(5),'a',a(5),'alpha',alpha(5));
L(6)=Link('d',d(6),'a',a(6),'alpha',alpha(6));
% 建立机器人
IRB120=SerialLink(L,'name','IRB120');

S1=d(1);
S2=a(1);
S3=a(2);
S4=a(3);
S5=d(4);
S6=d(6);

q=zeros(1,6);
x=FK(1,4);
y=FK(2,4);
z=FK(3,4);
i=[0 0 -S6]';
R=FK(1:3,1:3);
p6=[x y z]';
p5=p6+R*i;

% 计算第一个关节角
if p5(1)>=0&&p5(2)>=0
    q(1)=atan(p5(2)/p5(1));
elseif p5(1)<0&&p5(2)>=0
    q(1)=pi+atan(p5(2)/p5(1));
elseif p5(1)<=0&&p5(2)<0
    q(1)=-pi+atan(p5(2)/p5(1));
else
    q(1)=atan(p5(2)/p5(1));
end

% 计算第二、三关节角（利用余弦定理）
p2=[S2*cos(q(1)) S2*sin(q(1)) S1]';

b1=sqrt(S4^2+S5^2);
b2=sqrt((p2(1)-p5(1))^2+(p2(2)-p5(2))^2+(p2(3)-p5(3))^2);

beta1=acos((S3^2+b2^2-b1^2)/(2*S3*b2));
beta2=acos((b1^2+S3^2-b2^2)/(2*b1*S3));
beta3=acos((b1^2+b2^2-S3^2)/(2*b1*b2));

l=sqrt((p5(2)-p2(2))^2+(p5(1)-p2(1))^2);
q(2)=atan((p5(3)-p2(3))/l)+beta1;
q(3)=beta2+atan(S5/S4)-pi;

% 计算第五个关节角
T4=L(1).A(q(1))*L(2).A(q(2))*L(3).A(q(3));
R4=T4(1:3,1:3);
i4=-inv(R4)*(R*i);

theta5=atan(sqrt(i4(1)^2+i4(2)^2)/i4(3));
if (i4(1)<=0)&&(theta5>=0)
    q(5)=theta5;
elseif (i4(1)>0)&&(theta5>=0)
    q(5)=-theta5;
elseif (i4(1)<=0)&&(theta5<0)
    q(5)=pi+theta5;
else
    q(5)=-pi-theta5;
end


% 计算第四个关节角
if i4(1)>=0&&i4(2)>=0
    q(4)=atan(i4(2)/i4(1));
elseif i4(1)<0&&i4(2)>=0
    q(4)=pi+atan(i4(2)/i4(1));
elseif i4(1)<=0&&i4(2)<0
    q(4)=-pi+atan(i4(2)/i4(1));
else
    q(4)=atan(i4(2)/i4(1));
end

if q(4)==2*pi
    q(4)=q(4)-2*pi;
end

% 计算第六个关节角
T5=L(1).A(q(1))*L(2).A(q(2))*L(3).A(q(3))*L(4).A(q(4))*L(5).A(q(5));

T6=T5(1:3,1:3)\FK(1:3,1:3);

if T6(1,1)>=0&&T6(2,1)>=0
    q(6)=atan(T6(2,1)/T6(1,1));
elseif T6(1,1)<0&&T6(2,1)>=0
    q(6)=pi+atan(T6(2,1)/T6(1,1));
elseif T6(1,1)<=0&&T6(2,1)<0
    q(6)=-pi+atan(T6(2,1)/T6(1,1));
else
    q(6)=atan(T6(2,1)/T6(1,1));
end



end

