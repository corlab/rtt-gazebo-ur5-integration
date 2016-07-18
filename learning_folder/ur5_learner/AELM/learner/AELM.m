classdef AELM < ALearner
    %AELM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        hidDim = 10;    %number of neurons in the hidden layer
        Winp = [];      %weights from input layer to hidden layer
        Wout = [];      %weights from hidden layer to output layer
        b = [];         %bias parameters of activation functions
        a=1;            %experiment slope parameter for activation fct

        inpScale = 1;   %scale of uniform dist for weight init
        regOut = 1e-6;  %output regularization parameter for ridge regression
        g = 1;
        regInp = 1e-3;  %input weight regularization, negative value indicates that no regularization is performed
        inpTrained = 0; %helper to indicate whether input weights have been trained/regularized
        sp = [0 10 5];  %state prediction (radius, no. of trajs, no. of steps per traj, batch size) for stabilizing multi-stable dynamics

        batchSize = 1000; %number of samples used for a minibatch
    end
    
    methods
        function l = AELM(modDims, spec)
            l = l@ALearner(modDims, spec);
            %l.g = zeros(1,l.numMods);
            %for m=1:l.numMods
            %    l.g(m) = 1/l.modDims(m);
            %end
        end

        function init(l, X)          
            if isscalar(l.inpScale)
                l.Winp = l.inpScale .* (2 * rand(l.hidDim,l.inpDim) - ones(l.hidDim,l.inpDim));
            else
                l.Winp = zeros(l.hidDim,l.inpDim);
                for m=1:l.numMods
                    l.Winp(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = l.inpScale(m) .* (2 * rand(l.hidDim,l.modDims(m)) - ones(l.hidDim,l.modDims(m)));
                end
            end
            l.b = 2 * rand(1,l.hidDim) - ones(1,l.hidDim);
            l.Wout = 2 * rand(l.hidDim,l.outDim) - ones(l.hidDim,l.outDim);
            
            if nargin > 1 %&& l.BIP
                if iscell(X)
                    X = l.normalizeIO(cell2mat(X));
                else
                    X = l.normalizeIO(X);
                end
                %l.bip(X);
            end
        end
        
        function train(l, X, Y)
            if iscell(X)
                [X Y] = l.normalizeIO(cell2mat(X), cell2mat(Y));
            else
                [X Y] = l.normalizeIO(X, Y);
            end
            
            if min(l.regInp) >= 0
                l.trainInp(X, Y);
            end
            if l.sp(1) > 0
                l.trainOutSP(X, Y);
            else
                l.trainOut(X, Y);
            end
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
        
        function trainInp(l, X, Y)
            if size(X,1) < l.batchSize
                [hs as] = l.calcHiddenStates(X);
                if isscalar(l.regInp)
                    l.Winp = ((X'*X + l.regInp * eye(l.inpDim))\(X' * as))';
                else
                    for m=1:l.numMods
                        l.Winp(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = ...
                            ((X(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2))'*X(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) + l.regInp(m) * eye(l.modDims(m)))\(X(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2))' * as))';
                    end
                end
            else
                XTX = zeros(l.inpDim, l.inpDim);
                XTA = zeros(l.inpDim, l.hidDim);                
                numBatches = floor(size(X,1)/l.batchSize);
                rest = mod(size(X,1),l.batchSize);                
                for b=1:numBatches
                    Xtmp = X((b-1)*l.batchSize+1:b*l.batchSize,:);
                    [H A] = l.calcHiddenStates(Xtmp);
                    XTX = XTX + Xtmp' * Xtmp;
                    XTA = XTA + Xtmp' * A;
                end
                if rest > 0
                    Xtmp = X(numBatches*l.batchSize+1:end,:);
                    [H A] = l.calcHiddenStates(Xtmp);
                    XTX = XTX + Xtmp' * Xtmp;
                    XTA = XTA + Xtmp' * A;
                end                
                if isscalar(l.regInp)
                    XTX = XTX + diag(repmat(l.regInp, 1, l.inpDim));
                    l.Winp = (XTX\XTA)';
                else
                    for m=1:l.numMods
                        l.Winp(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) = ...
                            ((XTX(l.modBeginEnd(m,1):l.modBeginEnd(m,2),l.modBeginEnd(m,1):l.modBeginEnd(m,2)) + l.regInp(m) * eye(l.modDims(m)))\XTA(l.modBeginEnd(m,1):l.modBeginEnd(m,2),:))';
                    end
                end
            end
            l.inpTrained = 1;
        end

        function updateOut(l)
            h = l.calcHiddenStates(l.inp);
            l.out = h * l.Wout;
            %l.out = norm2range(l.out, l.outRange, l.outOffset);
        end
   
        function [H G] = calcHiddenStates(l, X)
            numSamples = size(X,1);
            
            %TODO this is extremely costly, make mex call for fermi/act fct
            %atemp = repmat(l.a, numSamples, 1);
            btemp = repmat(l.b, numSamples, 1);

            if l.inpTrained
                G = zeros(numSamples, l.hidDim);
                for m=1:l.numMods
                    G = G + l.g(m) .* (X(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2)) * l.Winp(:,l.modBeginEnd(m,1):l.modBeginEnd(m,2))');
                end
            else
                G = X * l.Winp';
            end
            %H = 1./(1+exp(-atemp .* G - btemp));
            H = 1./(1+exp(-G.*l.a - btemp));
        end

        function trainOutSP(l, X, Y)
            % l.sp = [radius #trajs #steps batchSize]
            
            % state prediction in joint IO space
            HTH = zeros(l.hidDim, l.hidDim);
            HTY = zeros(l.hidDim, l.outDim);
            numBatches = floor(size(X,1)/l.sp(4));
            rest = mod(size(X,1),l.sp(4)); 
            
            % vel. profile for sp trajs
            %fprintf('preparing contraction profile...');
            alpha = linspace(sqrt(l.sp(1)), 0, l.sp(3))';
            alpha = alpha.^2;
            alpha = [alpha; alpha(end)];
            alphaInp = []; %zeros(l.sp(4)*l.sp(2)*l.sp(3), l.outDim);
            alphaOut = [];
            for i=1:l.sp(4) % batch size
                for j=1:l.sp(2) % no trajs
                    for k=1:l.sp(3) % traj steps
                        alphaInp = [alphaInp; alpha(k)];
                        alphaOut = [alphaOut; alpha(k+1)];
                    end
                end
            end
            alphaInp = repmat(alphaInp, 1, l.inpDim);
            alphaOut = repmat(alphaOut, 1, l.outDim);
            %fprintf('done!\n');

            %fprintf([num2str(numBatches) ' mini batches']);
            for b=1:numBatches
                r = randn(l.sp(4)*l.sp(2), l.outDim);
                rn = repmat(sqrt(sum(r.^2,2)), 1, l.outDim);
                r = repmat(r ./ rn, l.sp(3), 1);
                Xtmp = repmat(X((b-1)*l.sp(4)+1:b*l.sp(4),:), l.sp(2) * l.sp(3), 1);
                Ytmp = Xtmp + alphaOut .* r;
                Xtmp = Xtmp + alphaInp .* r;
                
                H = l.calcHiddenStates(Xtmp);
                HTH = HTH + H' * H;
                HTY = HTY + H' * Ytmp;
                if mod(b,10) == 0
                    fprintf('.');
                end
            end
            alphaInp = alphaInp(1:rest*l.sp(2)*l.sp(3),:);
            alphaOut = alphaOut(1:rest*l.sp(2)*l.sp(3),:);
            if rest > 0
                r = randn(rest*l.sp(2), l.outDim);
                rn = repmat(sqrt(sum(r.^2,2)), 1, l.outDim);
                r = repmat(r ./ rn, l.sp(3), 1);
                Xtmp = repmat(X(numBatches*l.sp(4)+1:end,:), l.sp(2) * l.sp(3), 1);
                Ytmp = Xtmp + alphaOut .* r;
                Xtmp = Xtmp + alphaInp .* r;
                
                H = l.calcHiddenStates(Xtmp);
                HTH = HTH + H' * H;
                HTY = HTY + H' * Ytmp;
                fprintf('.');
            end
            %fprintf('done!\n');
            
            % compute output weights by regression
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
    
end

