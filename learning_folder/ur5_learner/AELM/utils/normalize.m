function [data offset range] = normalize(data)
%NORMALIZE Summary of this function goes here
%   Detailed explanation goes here

offset = mean(data, 1);
data = data - repmat(offset, size(data,1), 1);

range = [min(data, [], 1); max(data, [], 1)];
data = range2norm(data, range);

end
