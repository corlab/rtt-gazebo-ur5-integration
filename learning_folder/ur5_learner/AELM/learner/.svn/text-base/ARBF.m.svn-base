classdef ARBF < ALearner
    %AELM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hidDim = 0;    %number of neurons in the hidden layer
        Winp = [];      %weights from input layer to hidden layer
        Wout = [];      %weights from hidden layer to output layer
        d = [];
        h = [];
        
        hbdist = 0.1;
        c = 1;
        b = 1;
        regOut = 1e-6;  %output regularization parameter for ridge regression
        batchSize = 1000; %number of samples used for a minibatch
    end
    
    methods
        function l = ARBF(modDims, spec)
            l = l@ALearner(modDims, spec);
            
            if size(l.c,2) ~= l.inpDim
                l.c = l.c * ones(1,l.inpDim);
            end
        end

        function init(l, X)          
            if nargin > 1
                if iscell(X)
                    X = l.normalizeIO(cell2mat(X));
                else
                    X = l.normalizeIO(X);
                end
            end
        end
        
        function train(l, X, Y)
            if iscell(X)
                [X Y] = l.normalizeIO(cell2mat(X), cell2mat(Y));
            else
                [X Y] = l.normalizeIO(X, Y);
            end
            
            l.trainInp(X);
            l.trainOut(X, Y);
        end
        
        function trainOut(l, X, Y)
            if size(X,1) < l.batchSize
                hs = l.calcHiddenStates(X);
                if isscalar(l.regOut)
                    l.Wout = (hs'*hs + l.regOut * eye(l.hidDim))\(hs' * Y);
                else
                    hsths = hs'*hs;
                    for m=1:l.numMods
                        l.Wout(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = ...
                            (hsths + l.regOut(m) * eye(l.hidDim))\(hs' * Y(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)));
                    end
                end
            else
                HTH = zeros(l.hidDim, l.hidDim);
                HTY = zeros(l.hidDim, l.outDim);                
                numBatches = floor(size(X,1)/l.batchSize);
                rest = mod(size(X,1),l.batchSize);                
                for b=1:numBatches
                    H = l.calcHiddenStates(X((b-1)*l.batchSize+1:b*l.batchSize,:));
                    HTH = HTH + H' * H;
                    HTY = HTY + H' * Y((b-1)*l.batchSize+1:b*l.batchSize,:);
                end
                if rest > 0
                    H = l.calcHiddenStates(X(numBatches*l.batchSize+1:end,:));
                    HTH = HTH + H' * H;
                    HTY = HTY + H' * Y(numBatches*l.batchSize+1:end,:);
                end                
                if isscalar(l.regOut)
                    HTH = HTH + diag(repmat(l.regOut, 1, l.hidDim));
                    l.Wout = HTH\HTY;
                else
                    for m=1:l.numMods
                        l.Wout(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = ...
                            (HTH + l.regOut(m) * eye(l.hidDim))\HTY(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2));
                    end
                end
            end
        end
        
        function trainOutC(l, X, Y, C, B)
            if iscell(X)
                [X Y] = l.normalizeIO(cell2mat(X), cell2mat(Y));
            else
                [X Y] = l.normalizeIO(X, Y);
            end
            
            c = l.c;
            b = l.b;
            for m=1:l.numMods
                l.c = C(m,:);
                l.b = B(m);
                hs = l.calcHiddenStates(X);
                if isscalar(l.regOut)
                    l.Wout(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = (hs'*hs + l.regOut * eye(l.hidDim))\(hs' * Y(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)));
                else
                    l.Wout(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = ...
                        (hs'*hs + l.regOut(m) * eye(l.hidDim))\(hs' * Y(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)));
                end
            end
            l.c = c;
            l.b = b;
        end
        
        function trainInp(l, X)
            %place centers
            numSamples = size(X,1);
            p = randperm(numSamples);
            for i=1:numSamples
                if isempty(l.Winp)
                    l.Winp(size(l.Winp,1)+1,:) = X(p(i),:);
                else
                    [idx d] = l.nearestNeighbors(X(p(i),:), 1);
                    if d(1) > l.hbdist
                        l.Winp(size(l.Winp,1)+1,:) = X(p(i),:);
                    end
                end
            end
            
            %reset network size
            l.hidDim = size(l.Winp,1);
            l.d = zeros(l.hidDim,1);
            l.h = zeros(l.hidDim,1);
        end
        
        function updateOut(l)
            l.d = l.distance(l.Winp, l.inp);

            l.h = smaxFct(-l.b .* l.d);
     
            l.out = (l.Wout' * l.h)';
        end
   
        function [H] = calcHiddenStates(l, X)
            H = zeros(size(X,1), l.hidDim);
            for i=1:size(X,1)
            	l.d = l.distance(l.Winp, X(i,:));
                l.h = smaxFct(-l.b .* l.d);
                H(i,:) = l.h';
            end
        end
        
        function [n d] = nearestNeighbors(l, x, K)
            d = l.distance(l.Winp, x);
            [d n] = sort(d, 1, 'ascend');
            if nargin > 2
                d = d(1:K);
                n = n(1:K);
            end
        end
        
        function [d] = distance(l, a, b)
            %if exist('mexARBFdistance.mexglx')
                %d = mexWeightedSqEucDistance(a, b, l.c);
            %else
                numSamplesA = size(a,1);
                numSamplesB = size(b,1);

                if numSamplesA == numSamplesB
                    d = sum( l.c .* (a - b).^2, 2);
                else
                    d = sum( repmat(l.c, numSamplesA, 1) .* (a - repmat(b, numSamplesA, 1)).^2, 2);
                end
            %end
        end
       
    end
    
end

