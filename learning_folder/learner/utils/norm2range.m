function [y] = norm2range(x, ra, offset, minmax)
%NORM2RANGE y = norm2range(x, ra, minmax)
%   ra is a dim x 2 matrix holding min and max values per dimension
%   minmax is repmat([-1 1], dim, 1) by default

if nargin < 3
    offset = zeros(1, size(x,2));
end

if nargin < 4
    minmax = repmat([-1 1]', 1, size(x,2));
end

%cut exceeding dimensions
ra = ra(:,1:min(size(ra,2), size(x,2)));
x = x(:,1:min(size(ra,2), size(x,2)));

tmp = minmax(2,:) - minmax(1,:);
tmp(tmp==0) = 1;
scale = 1./(tmp);
range = (ra(2,:) - ra(1,:));

y = zeros(size(x));
for i=1:size(x,1)
    y(i,:) = (scale .* (x(i,:) - minmax(1,:))) .* range + ra(1,:);
    y(i,:) = y(i,:) + offset;
end
