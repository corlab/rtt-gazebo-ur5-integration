function [T] = space2time(X, dim)
%SPACE2TIME Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    dim = 2;
end

dimsteps = size(X,2);
steps = dimsteps/dim;

if isvector(X)
    T = zeros(steps,dim);
    for d=1:dim
        T(:,d) = X((d-1)*steps+1:d*steps);
    end
else
    T = cell(size(X,1),1);
    for i=1:length(T)
        T{i} = zeros(steps,dim);
        for d=1:dim
            T{i}(:,d) = X(i,(d-1)*steps+1:d*steps);
        end
    end
end

end

