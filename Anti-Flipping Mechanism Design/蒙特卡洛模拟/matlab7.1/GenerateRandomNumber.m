function [ q, qd, qdd ,p0dd ] = GenerateRandomNumber( q_range, qd_range, qdd_range, p0dd_range )
% 构建随机数
    q=zeros(6,1);
    qd=zeros(6,1);
    qdd=zeros(6,1);
    p0dd=zeros(3,1);
    for i=1:6
        q(i)=-q_range(i)+2*q_range(i)*rand;
        qd(i)=-qd_range(i)+2*qd_range(i)*rand;
%         qdd(i)=-qdd_range(i)+2*qdd_range(i)*rand;
        qdd(i)=qdd_range(i);
    end
    p0dd(1)=-p0dd_range(1)+2*p0dd_range(1)*rand;
    p0dd(2)=-p0dd_range(2)+2*p0dd_range(2)*rand;
    p0dd(3)=p0dd_range(3);
end

