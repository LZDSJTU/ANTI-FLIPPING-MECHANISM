function [ threshold ] = confidentRadius( a, f, Xi )
%UNTITLED ????????????
%   ????????
add = (Xi(2)-Xi(1))*(f(2)-f(1))/2;
flag = 0;
for i = 2:size(Xi,2)
    add = add + (Xi(i+1)-Xi(i))*f(i);
    if add > 1-a;
        flag = 1;
        threshold = Xi(i);
        break
    end
end
if flag == 0
    threshold = 0;
end

end

