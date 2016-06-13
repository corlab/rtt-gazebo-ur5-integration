function [x] = tr2cr(y, dim, numCells, minval, maxval)
%TR2CR Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    numCells = 50;
end

K = size(y,1);

if nargin < 4
    minval = -1;
    maxval = 1;
end

x = zeros(K, dim);

v = linspace(minval, maxval, numCells);
V = [];
if dim == 2
    V = zeros(dim, numCells^2);
    cnt = 1;
    for i=1:numCells
        for j=1:numCells
            V(1,cnt) = v(i);
            V(2,cnt) = v(j);
            cnt = cnt + 1;
        end
    end
elseif dim == 3
    V = zeros(dim, numCells^3);
    cnt = 1;
    for i=1:numCells
        for j=1:numCells
            for k=1:numCells
                V(1,cnt) = v(i);
                V(2,cnt) = v(j);
                V(3,cnt) = v(k);
                cnt = cnt + 1;
            end
        end
    end
end

for k=1:K
    if isscalar(numCells)
        if dim == 1
            tmp = y(k,:);
            tmp = tmp/sum(tmp);
            x(k) = sum(v .* tmp);
        elseif dim == 2
            tmp = y(k,:);
            tmp = tmp/sum(tmp);
            x(k,1) = sum(V(1,:) .* tmp);
            x(k,2) = sum(V(2,:) .* tmp);
        elseif dim == 3
            tmp = y(k,:);
            tmp = tmp/sum(tmp);
            x(k,1) = sum(V(1,:) .* tmp);
            x(k,2) = sum(V(2,:) .* tmp);
            x(k,3) = sum(V(3,:) .* tmp);
        else
            %TODO
        end
    else
        %TODO
    end
end

end

