classdef LinearMapping < Learner
    %LINEARMAPPING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        W = [];
        b = [];
        
        reg = 1e-6;     %regularization parameter for ridge regression
    end
    
    methods
        function l = LinearMapping(inpDim, outDim, spec)
            l = l@Learner(inpDim, outDim, spec);
        end
        
         function init(l, X)
            l.W = 2 * rand(l.inpDim, l.outDim) - ones(l.inpDim, l.outDim);
            l.b = zeros(1,l.outDim);
        end
        
        function train(l, X, Y)
            if iscell(X)
                X = cell2mat(X);
                Y = cell2mat(Y);
            end
            
            X = [X ones(size(X,1), 1)];
            W = ((X' * X + l.reg * eye(size(X,2))) \ (X' * Y));

            l.W = W(1:end-1,:);
            l.b = W(end,:);
        end
        
        function [Y] = apply(l, X)
            if iscell(X)
                Y = cell(length(X),1);
                for i=1:length(X)
                    Y{i} = l.apply(X{i});
                end
            else
                Y = X * l.W + repmat(l.b, size(X,1), 1);
                l.out = Y(end,:);
            end
        end
    end
    
end

