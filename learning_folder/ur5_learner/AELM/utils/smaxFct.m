function [Y] = smaxFct(X)
%SMAXFCT Summary of this function goes here
%   Detailed explanation goes here

% if isvector(X)
%     Y = softmax(X);
% else
%     Y = softmax(X')';
% end

if isvector(X)
    ex = exp(X);
    de = sum(ex);
    Y = ex./(de + (de == 0));
else
    ex = exp(X);
    de = repmat(sum(ex,1), size(X,1), 1);
    Y = ex./(de + (de == 0));
end
