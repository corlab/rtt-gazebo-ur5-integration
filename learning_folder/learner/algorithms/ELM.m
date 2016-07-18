classdef ELM < Learner
    %ELM Extreme Learning Machine
    %   Single layer feed-forward network
    
    properties
        hidDim = 10;    %number of neurons in the hidden layer
        wInp = [];      %weights from input layer to hidden layer
        wOut = [];      %weights from hidden layer to output layer
        a = [];         %slope parameters of activation functions
        b = [];         %bias parameters of activation functions
        
        reg = 1e-6;     %regularization parameter for ridge regression
        BIP = 1;        %flag for using Batch Intrinsic Plasticity (BIP)
        mu = 0.2;       %desired mean activity parameter for BIP

        batchSize = 5000; %number of samples used for a minibatch
        
        confest=[];
    end
    
    methods

        function l = ELM(inpDim, outDim, spec)
            if nargin==0
                inpDim=1;
                outDim=1;
                spec={};
            end
            l = l@Learner(inpDim, outDim, spec);
        end
        
 
        
        function init(l, X, Y)
            l.a = ones(1,l.hidDim);
            l.b = 2 * rand(1,l.hidDim) - ones(1,l.hidDim);
            l.wInp = 2 * rand(l.hidDim,l.inpDim) - ones(l.hidDim,l.inpDim);
            l.wOut = 2 * rand(l.hidDim,l.outDim) - ones(l.hidDim,l.outDim);
            
            l.inpOffset=[];
            l.inpRange=[];
            l.outOffset=[];
            l.outRange=[];
            
            
            if nargin < 3
                Y=[];
            end
            
            if nargin > 1
                if iscell(X)
                    X = l.normalizeIO(cell2mat(X), cell2mat(Y));
                else
                    X = l.normalizeIO(X, Y);
                end
            end
 
            
            if  l.BIP
                l.bip(X);
            end
        end
        
        function train(l, X, Y)
            if iscell(X)
                [X Y] = l.normalizeIO(cell2mat(X), cell2mat(Y));
            else
                [X Y] = l.normalizeIO(X, Y);
            end
            
            if size(X,1) < l.batchSize
                hs = l.calcHiddenStates(X);
                l.wOut = (hs'*hs + l.reg * eye(l.hidDim))\(hs' * Y);
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
                HTH = HTH + diag(repmat(l.reg, 1, l.hidDim));
                l.wOut = HTH\HTY;
            end
        end
        
        function [Y] = apply(l, X)
            conf=[];
            if iscell(X)
                Y = cell(length(X),1);
                for i=1:length(X)
                    Y{i} = l.apply(X{i});
                end
            else
                X = range2norm(X, l.inpRange, l.inpOffset);
                if size(X,1) < l.batchSize
                    H = l.calcHiddenStates(X);
                    Y = H * l.wOut;
                    
                    
                else
                    Y = zeros(size(X,1), l.outDim);
                    numBatches = floor(size(X,1)/l.batchSize);
                    rest = mod(size(X,1),l.batchSize);
                    for b=1:numBatches
                        H = l.calcHiddenStates(X((b-1)*l.batchSize+1:b*l.batchSize,:));
                        Y((b-1)*l.batchSize+1:b*l.batchSize,:) = H * l.wOut;
                    end
                    if rest > 0
                        H = l.calcHiddenStates(X(numBatches*l.batchSize+1:end,:));
                        Y(numBatches*l.batchSize+1:end,:) = H * l.wOut;
                    end
                end
                Y = norm2range(Y, l.outRange, l.outOffset);
                l.out = Y(end,:);
            end
        end
        
        
        function [H G] = calcHiddenStates(l, X)
            numSamples = size(X,1);
            
            %TODO this is extremely costly, make mex call for fermi fct
            atemp = repmat(l.a, numSamples, 1);
            btemp = repmat(l.b, numSamples, 1);
            %

            G = X * l.wInp';
    
            H = 1./(1+exp(-atemp .* (G - btemp)));
        end
        
  
        
        %batch intrinsic plasticity
        function bip(l, X)
            numSamples = size(X,1);
            
            PD = ProbDistUnivParam('exponential', l.mu);
            G = X * l.wInp';

            for hn = 1:l.hidDim
                targets = random(PD, 1, numSamples);

                hightars = length([targets(targets>=1),targets(targets<=0)]);
                while hightars > 0
                    further_targets = random(PD, 1, hightars);
                    targets = [targets(targets<1&targets>0),further_targets];
                    hightars = length([targets(targets>=1),targets(targets<=0)]);
                end
                targets = sort(targets);
                
                s = sort(G(:,hn));
                Phi = [s,ones(numSamples,1)];
                targetsinv = -log(1./targets - 1); %apply inverse activation function
                
                w = pinv(Phi)*targetsinv';                
                
                l.a(hn) = w(1);
                l.b(hn) = w(2);
            end
        end
    end
    
end

