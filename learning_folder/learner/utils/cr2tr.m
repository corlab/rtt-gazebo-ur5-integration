function [y] = cr2tr(x, numCells, minval, maxval, sigma)
%CR2TR Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    numCells = 50;
end

if nargin < 4
    minval = -1;
    maxval = 1;
end

if nargin < 5
    sigma = 0.01;
end

K = size(x,1);
dim = size(x,2);

y = zeros(K, numCells * dim);

v = linspace(minval, maxval, numCells);

for k=1:K
    if isscalar(numCells)
        if dim == 1
            for i=1:numCells
                y(k,i) = exp(-(1/sigma) * (v(i) - x(k)).^2);
            end
        elseif dim == 2
            cnt = 1;
            for i=1:numCells
                for j=1:numCells
                    y(k,cnt) = exp(-(1/sigma) * ((v(i) - x(k,1)).^2 + (v(j) - x(k,2)).^2));
                    cnt = cnt + 1;
                end
            end
        elseif dim == 3
            cnt = 1;
            for i=1:numCells
                for j=1:numCells
                    for l=1:numCells
                        y(k,cnt) = exp(-(1/sigma) * ((v(i) - x(k,1)).^2 + (v(j) - x(k,2)).^2 + (v(l) - x(k,3)).^2));
                        cnt = cnt + 1;
                    end
                end
            end
        else
            %TODO
        end
    else
        %TODO
    end
end


end

