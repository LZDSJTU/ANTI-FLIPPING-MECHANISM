function [ ] = printData( intersect, nn )
% Print Data Intersection Point Info
fid = fopen(['intersect' num2str(nn) '.txt'],'w');
fprintf(fid,'%1$3s%2$9s%3$9s %4$54s %5$54s %6$54s\n',...
    'x','y', 'q', 'qd', 'qdd', 'p0dd');
[m,n] = size(intersect);
for i = 1:m
    for j = 1:n
        if j == n
            fprintf(fid,'%9.5f\n',intersect(i,j));
        else
            fprintf(fid,'%9.5f',intersect(i,j));
        end
    end
end
fclose(fid);
end

