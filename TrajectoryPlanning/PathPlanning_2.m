clear
clc
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

T1=transl(0.374,0,0.464)*troty(pi/2)*trotz(pi);
T2=transl(0.65,0,0.1)*troty(pi/2)*trotz(pi);
T3=transl(0.15,0,0.3)*troty(pi/2)*trotz(pi);

t1=0:0.001:0.5;
Ts1 = ctraj(T1, T2, length(t1));
n=0;
for i=t1
   n=n+1;
   q(n,:)=invkine(Ts1(:,:,n));
end

m=n;
t2=0.5:0.001:1;
Ts2 = ctraj(T2, T3, length(t2));
n=n-1;
for i=t2
   n=n+1;
   q(n,:)=invkine(Ts2(:,:,n-m+1));
end

t=[t1, t2];
t(length(t1))=[];
for i=1:6
   qd(:,i)=diff(q(:,i)')./diff(t);
   qdd(:,i)=diff(qd(:,i)')./diff(t(2:length(t)));
end
q(1:3,:)=[];
qd(1:2,:)=[];
qdd(1,:)=[];
t(1:3)=[];
t=t';
plot(t,q(:,2));
hold on;
plot(t,qd(:,2));
hold on;
plot(t,qdd(:,2));

% save('PathPlanning_2.mat','q','qd','qdd');

% t=t';
% q1=[t,q(:,1)];
% q2=[t,q(:,2)-pi/2];
% q3=[t,q(:,3)];
% q4=[t,q(:,4)];
% q5=[t,q(:,5)];
% q6=[t,q(:,6)];
% figure(1);
% plot(q1(:,1),q1(:,2));
% figure(2);
% plot(q2(:,1),q2(:,2));
% figure(3);
% plot(q3(:,1),q3(:,2));
% figure(4);
% plot(q4(:,1),q4(:,2));
% figure(5);
% plot(q5(:,1),q5(:,2));
% figure(6);
% plot(q6(:,1),q6(:,2));
% 
% save('displacement2_1.txt','q1','-ascii');
% save('displacement2_2.txt','q2','-ascii');
% save('displacement2_3.txt','q3','-ascii');
% save('displacement2_4.txt','q4','-ascii');
% save('displacement2_5.txt','q5','-ascii');
% save('displacement2_6.txt','q6','-ascii');

% figure(7);
% s=[-1.2 0.8 -0.8 0.8 -0.3 0.8];
% IRB120.plot(q,'workspace',s,'fps',100);




