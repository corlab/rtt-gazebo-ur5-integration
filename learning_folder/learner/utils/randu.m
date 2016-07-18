function [ R ] = randu(rows, cols, range)
%RANDU Summary of this function goes here
%   Detailed explanation goes here

if nargin < 3
    range = 1;
end

R = range * 2.0 * (rand(rows, cols) - 0.5);

end

