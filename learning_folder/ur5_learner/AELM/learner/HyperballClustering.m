classdef HyperballClustering < ClusteringAlg
    %METRIC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hbdist = 0.1;
    end
    
    methods
        function alg = HyperballClustering(hbdist)
            alg = alg@ClusteringAlg();
            
            if nargin < 1
                hbdist = 0.1;
            end
            alg.hbdist = hbdist;
        end
        
        function C = cluster(alg, X)
            %place centers
            numSamples = size(X,1);
            p = randperm(numSamples);
            for i=1:numSamples
                if isempty(alg.C)
                    alg.C = X(p(i),:);
                else
                    [idx d] = alg.nearestNeighbors(X(p(i),:), 1);
                    if d(1) > alg.hbdist
                        alg.C = [alg.C; X(p(i),:)];
                    end
                end
            end
        end
    end
    
end

