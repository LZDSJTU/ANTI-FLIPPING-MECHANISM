function [ IRB120, L ] = CreateRobot( d, a, alpha, n )
% 利用matlab toolbox构建IRB120机械臂
% 建立IRB120的D-H参数矩阵
for i = n
    L(i)=Link('d',d(i),'a',a(i),'alpha',alpha(i));
end
% 建立机器人
IRB120=SerialLink(L,'name','IRB120','tool',[eye(3),[0;0;0.072];0,0,0,1]);
end

