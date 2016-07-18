classdef ClusteringAlg < handle
    %METRIC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        metric = [];
        C = [];
    end
    
    methods
        function alg = ClusteringAlg()
            alg.metric = SqEucDistance();
        end
        
        function C = cluster(alg, X)
            C = alg.C;
        end
        
        function [n d] = nearestNeighbors(alg, x, K)
            d = alg.metric.distance(repmat(x,size(alg.C,1),1), alg.C);
            [d n] = sort(d, 1, 'ascend');
            if nargin > 2
                d = d(1:K);
                n = n(1:K);
            end
        end
        
        function [n d] = knn(alg, X, w, K)
            d = alg.metric.distance(X, repmat(w,size(X,1),1));
            [d n] = sort(d, 1, 'ascend');
            if nargin > 2
                d = d(1:K);
                n = n(1:K);
            end
        end
        
        function fitClustersToData(alg, X)
            for i=1:size(alg.C,1)
                [idx d] = alg.knn(X, alg.C(i,:), 1);
                alg.C(i,:) = X(idx,:);
            end
        end
        
        function new = copy(this)
            new = feval(class(this));
            p = properties(this);
            for i = 1:length(p)
                if( isa(this.(p{i}), 'handle'))
                    new.(p{i}) = this.(p{i}).copy();
                elseif isa(this.(p{i}),'cell')
                    new.(p{i}) = deepCopyCellArray(numel(this.(p{i})));
                else
                    new.(p{i}) = this.(p{i});
                end
            end
        end
    end
    
end

