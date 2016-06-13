function [y] = range2norm(x, ra, offset, minmax)
%RANGE2NORM y = range2norm(x, ra, minmax)
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

tmp = ra(2,:) - ra(1,:);
tmp(tmp==0) = 1;
scale = 1 ./ (tmp);

y = zeros(size(x));
for i=1:size(x,1)
    y(i,:) = x(i,:) - offset;
    %y(i,:) = 2 * (((y(i,:) - ra(1,:)) .* scale) - 0.5);
    y(i,:) = ((minmax(2,:) - minmax(1,:)) .* ((y(i,:) - ra(1,:)) .* scale)) + minmax(1,:);
end
