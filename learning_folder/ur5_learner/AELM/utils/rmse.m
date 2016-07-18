function [ err ] = rmse( X, Y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    D = X - Y;
    % average error over dimensions
    MSE = sum(D.^2, 2);
    RMSE = sqrt(MSE)/ size(X, 2);
    % average error of samples
    err = mean(RMSE);

end

