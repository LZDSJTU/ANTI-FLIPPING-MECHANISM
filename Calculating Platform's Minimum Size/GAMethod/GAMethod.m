clear;clc;
tic
% ����ÿ���������Ͻ����½磨��ȡֵ��Χ��
lb=-[165*pi/180,110*pi/180,110*pi/180,250*pi/180,250*pi/180,250*pi/180,2000*pi/180,2000*pi/180,2000*pi/180]';
ub=[165*pi/180,110*pi/180,110*pi/180,250*pi/180,250*pi/180,250*pi/180,2000*pi/180,2000*pi/180,2000*pi/180]';
                                                                                                                                                        
% �����ʽ�ľ���
Ae=[];
% �����ʽ������
be=[];
% ���岻��ʽ�ľ���
As=[];
% ���岻��ʽ������
bs=[];
% �Ŵ��㷨���
[Parameters,fval,exitflag]=ga(@CalculateDiameter,9,As,bs,Ae,be,lb,ub);

q=[Parameters(1:3), zeros(1,3)];
% D-H�����������������ʱ��ʵ����ֵ����
d=[0.124; 0.0; 0.0; 0.302; 0; 0.072];
a=[0.0; 0.27; 0.07; 0.0; 0.0; 0.0;];
alpha=[pi/2; 0; pi/2; pi/2; -pi/2; 0];
% ����IRB120��D-H��������
L(1)=Link('d',d(1),'a',a(1),'alpha',alpha(1));
L(2)=Link('d',d(2),'a',a(2),'alpha',alpha(2));
L(3)=Link('d',d(3),'a',a(3),'alpha',alpha(3));
L(4)=Link('d',d(4),'a',a(4),'alpha',alpha(4));
L(5)=Link('d',d(5),'a',a(5),'alpha',alpha(5));
L(6)=Link('d',d(6),'a',a(6),'alpha',alpha(6));
% ����������
IRB120=SerialLink(L,'name','IRB120');
figure(1)
s=[-1.2 0.8 -0.8 0.8 -0.3 0.8];
IRB120.plot(q,'workspace',s,'fps',100);
toc

    
    

