function [ IRB120, L ] = CreateRobot( d, a, alpha, n )
% ����matlab toolbox����IRB120��е��
% ����IRB120��D-H��������
for i = n
    L(i)=Link('d',d(i),'a',a(i),'alpha',alpha(i));
end
% ����������
IRB120=SerialLink(L,'name','IRB120','tool',[eye(3),[0;0;0.072];0,0,0,1]);
end
