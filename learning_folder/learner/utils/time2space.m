function [X] = time2space(T)
%TIME2SPACE Summary of this function goes here
%   Detailed explanation goes here

if iscell(T)
    X = zeros(length(T),prod(size(T{1})));
    for i=1:length(T)
        X(i,:) = T{i}(:)';
    end
else
    X = T(:)';
end

end

