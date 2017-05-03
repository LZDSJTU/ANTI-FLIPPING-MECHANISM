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

T1=troty(pi/2)*trotz(pi)*transl(0.25,0.2,0.3);
T2=troty(pi/2)*trotz(pi)*transl(0.25,-0.2,0.3);

t=0:0.01:5;
Ts = ctraj(T1, T2, length(t));
% plot(t, transl(Ts));
n=0;
for i=t
   n=n+1;
   q(n,:)=invkine(Ts(:,:,n));
end

for i=1:6
%    figure(i);
%    qd(:,i)=diff(q(:,i)')./diff(t);
%    qdd(:,i)=diff(qd(:,i)')./diff(t(2:length(t)));
   
%    coef(i,:)=polyfit(t(1:length(t))',q(:,i),5);
%    x_hat(i,:)=t(1:length(t))';
%    y_hat(i,:)=polyval(coef(i,:),x_hat(i,:));
%    
%    plot(t(1:length(t)),q(:,i),'*',x_hat(i,:),y_hat(i,:),'-');
%    
   
%    coef(i,:)=polyfit(t(2:length(t))',qd(:,i),5);
%    x_hat(i,:)=t(2:length(t))';
%    y_hat(i,:)=polyval(coef(i,:),x_hat(i,:));
%    plot(t(2:length(t)),qd(:,i),'*',x_hat(i,:),y_hat(i,:),'-');
%    plot(x_hat(i,:),y_hat(i,:),'-');

%    coef(i,:)=polyfit(t(3:length(t))',qdd(:,i),10);
%    x_hat(i,:)=t(3:length(t))';
%    y_hat(i,:)=polyval(coef(i,:),x_hat(i,:));
%    plot(t(3:length(t)),qdd(:,i),'*',x_hat(i,:),y_hat(i,:),'-');
%       plot(x_hat(i,:),y_hat(i,:),'-');
end
% 

% t=t';
% q1=[t,q(:,1)];
% q2=[t,q(:,2)];
% q3=[t,q(:,3)];
% q4=[t,q(:,4)];
% q5=[t,q(:,5)];
% q6=[t,q(:,6)];

% save('displacement1.txt','q1','-ascii');
% save('displacement2.txt','q2','-ascii');
% save('displacement3.txt','q3','-ascii');
% save('displacement4.txt','q4','-ascii');
% save('displacement5.txt','q5','-ascii');
% save('displacement6.txt','q6','-ascii');
% 
% for i=1:6
%     coef1(:,i)=coef(:,7-i);
% end



figure(7)
s=[-1.2 0.8 -0.8 0.8 -0.3 0.8];
IRB120.plot(q,'workspace',s,'fps',100);



