function [data offset range] = normalize(data, offset, range)
%NORMALIZE Summary of this function goes here
%   Detailed explanation goes here

if nargin<2
    offset = mean(data, 1);
end
data = data - repmat(offset, size(data,1), 1);

if nargin<3
    range = [min(data, [], 1); max(data, [], 1)];
end

data = range2norm(data, range);

end
