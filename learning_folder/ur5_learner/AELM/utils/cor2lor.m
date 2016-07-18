function [y] = cor2lor(x, numCells, minval, maxval, sigma)
%COR2LOR Summary of this function goes here
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

if length(numCells) == 1
    y = zeros(K, numCells * dim);
    v = linspace(minval, maxval, numCells);

    for k=1:K
        for d=1:dim
            o = (d-1) * numCells;
            for i=1:numCells
                y(k,i+o) = exp(-(1/sigma) * (v(i) - x(k,d)).^2);
            end
        end 
    end
else
    y = zeros(K, sum(numCells));
    numCellsDim = 0;
    for d=1:dim
        v = linspace(minval(d), maxval(d), numCells(d));

        for k=1:K
            for i=1:numCells(d)
                y(k,i+numCellsDim) = exp(-(1/sigma) * (v(i) - x(k,d)).^2);
            end
        end
        numCellsDim = numCellsDim + numCells(d);
    end
end

end

