function  R  = HomogeneousMatrix( L, q, n )
% 
    T(1:4,1:4,1)=L(1).A(q(1));
    R(1:3,1:3,1)=T(1:3,1:3,1);
    for i=2:n
        RMiddleValue=L(i).A(q(i));
        R(1:3,1:3,i)=RMiddleValue(1:3,1:3);
        T(:,:,i)=T(:,:,i-1)*RMiddleValue;
    end

end

