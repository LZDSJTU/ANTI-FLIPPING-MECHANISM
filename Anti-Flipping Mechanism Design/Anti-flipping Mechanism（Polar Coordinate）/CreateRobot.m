function [ IRB120, L ] = CreateRobot( d, a, alpha, n )
% ����matlab toolbox����IRB120��е��
% ����IRB120��D-H��������
for i = 1:n
    L(i)=Link('d',d(i),'a',a(i),'alpha',alpha(i));
end
% ����������
IRB120=SerialLink(L,'name','IRB120');
end

