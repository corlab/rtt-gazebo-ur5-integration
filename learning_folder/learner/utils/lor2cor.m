function [x] = lor2cor(y, numCells, minval, maxval)
%LOR2COR Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    numCells = 50;
end

if nargin < 4
    minval = -1;
    maxval = 1;
end

threshold = 1e-4;


K = size(y,1);
if length(numCells) == 1
    dim = size(y,2)/numCells;
else
    dim = length(numCells);
end

if mod(dim,1)
    error('bad dimensions');
end

x = zeros(K, dim);

if length(minval) == 1
    v = linspace(minval, maxval, numCells);
    dx = (maxval - minval)/(numCells-1);

    for k=1:K
        for d=1:dim
            %binarize
            %x(k,d) = mean(v( y(k, (d-1)*numCells+1:d*numCells) > threshold));

            %smooth
            tmp = y(k, (d-1)*numCells+1:d*numCells);
            tmp = tmp/sum(tmp);
            x(k,d) = sum(v .* tmp);
        end
    end
else
    numCellsDim = 0;
    for d=1:dim
        v = linspace(minval(d), maxval(d), numCells(d));
        dx = (maxval(d) - minval(d))/(numCells(d)-1);

        for k=1:K
            tmp = y(k, numCellsDim+1:numCellsDim+numCells(d));
            tmp = tmp/sum(tmp);
            x(k,d) = sum(v .* tmp);
        end
        numCellsDim = numCellsDim + numCells(d);
    end
end

end

