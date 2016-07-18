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
            l = l@Learner(inpDim, outDim, spec);
        end

        function init(l, X)
            l.a = ones(1,l.hidDim);
            l.b = 2 * rand(1,l.hidDim) - ones(1,l.hidDim);
            l.wInp = 2 * rand(l.hidDim,l.inpDim) - ones(l.hidDim,l.inpDim);
            l.wOut = 2 * rand(l.hidDim,l.outDim) - ones(l.hidDim,l.outDim);
            
            if nargin > 1 && l.BIP
                if iscell(X)
                    X = l.normalizeIO(cell2mat(X));
                else
                    X = l.normalizeIO(X);
                end
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
                l.confest = inv(hs'*hs + l.reg * eye(l.hidDim));
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
        
        function [Y, conf] = apply(l, X)
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
                    
                    if ~isempty(l.confest)
                        warning('calculating reliability is not correct ?!');
                        %size(H)
                        %size(l.confest)
                        %size(l.wOut)
                        %conf = abs(H)*l.confest*l.wOut;
                        conf = zeros(size(H,1),1);
                        %due to http://reliawiki.org/index.php/Multiple_Linear_Regression_Analysis
                        % yp= t_a/2,n.. sqrt(sigma(Ys)*xp'*(X'X)^-1*xp
                        for ci=1:size(H,1)
                            conf(ci)=sqrt(abs(H(ci,:)*l.confest*H(ci,:)'));
                        end
                    end
                    
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
        
        
        %calculating d ELMffwd(in) / d in
        function [x_error, y_error]=propagate_error_old(l, X, Y)

            
            [H G] = l.calcHiddenStates(X);
            Y_net=H*l.wOut;
            
            Y_delta=Y_net-Y
            wout_delta = l.wOut*Y_delta';
            H_delta=wout_delta'.*l.a.*(H.*(1-H));
            X_delta=H_delta*l.wInp;
            
            x_error=-X_delta;
            y_error=sqrt(sum(Y_delta.^2));
            
        end
        
        
        %func: out=f(a.*(x*wInp+b))*wOut deriviative:
        %f'(a.*(x*wInp*+b))*wInp*wOut*a
        function J=jacobian_noscale(l, X)
            [H G] = l.calcHiddenStates(X);

            %alterative: (not working properly):
            %%J = ( l.wOut'* ((l.wInp'*diag(H.*(1-H)))') );

            %Hmat = diag(H.*(1-H));
            Hvec=H.*(1-H);
            Hvec=Hvec.*l.a;
            HH = repmat(Hvec,size(l.wInp,2),1)';
            HHH = l.wInp.*HH;
            J = l.wOut' * HHH; 
        end
        
        
        

        function J=jacobian(l, X)

            %input scaling
            X = range2norm(X, l.inpRange, l.inpOffset);
            
            J=jacobian_noscale(l, X);

            %output scaling:
            range = (l.outRange(2,:) - l.outRange(1,:)).*(1./(l.inpRange(2,:) - l.inpRange(1,:)));
            %range = 0.5.*(l.outRange(2,:) - l.outRange(1,:));
            range = repmat(range',1,size(J,2));

            J=J.*range;

        end
        

        
        
        function [x_error, y_error]=propagate_error(l, X, Y, lambda, minmaxbounds)
            
            J=l.jacobian_noscale(X);
            

            
            %iksurvey.pdf:
            [H G] = l.calcHiddenStates(X);
            
            Y_net=H*l.wOut;
            
            %if at bounds: jacobian is 0:
            if (nargin>4) && (~isempty(minmaxbounds))
                mask = (Y_net<minmaxbounds(:,1)') | (Y_net>minmaxbounds(:,2)');
                J(mask,:)=0;
            end
            
            
            
            
            
            Y_delta=Y_net-Y;
            X_delta= (J'/(J*J'+(lambda.^2)*eye(size(J,1)))) ; %least squares + damping
            X_delta=X_delta*Y_delta';
            %X_delta = pinv(J)*Y_delta;
            
            X_delta(isnan(X_delta))=0;

            
            x_error=-X_delta;
            y_error=sqrt(sum(Y_delta.^2));
        end
            
            
            
            
        function [X_est, Y_est, Y_err] = inverse_search(l, Y_des, X_init, minmaxbounds)
            %do fancy gradient descent 8-)
            
            err=inf;
            iteration=0;
            maxiterations=250;
            miniterations=5;
            minerror=1e-3;
            
            X_curr = range2norm(X_init, l.inpRange, l.inpOffset);
            if ~(isempty(l.outRange) || isempty(l.outOffset))
                Y_int = range2norm(Y_des, l.outRange, l.outOffset);
            else
                Y_int=Y_des;
            end
            
            if nargin>3
                if ~(isempty(l.outRange) || isempty(l.outOffset))
                    minmaxbounds=[range2norm(minmaxbounds(:,1), l.outRange, l.outOffset),range2norm(minmaxbounds(:,1), l.outRange, l.outOffset)];
                else
                    %minmaxbounds=minmaxbounds;
                end
            else
                minmaxbounds=[];
            end
            
            
            
            while (miniterations>iteration)||((err>minerror) && (iteration<maxiterations))
  
                
                
                iteration=iteration+1;
                
                %[x_error, y_error]=l.propagate_error(X_curr, Y_int);
                [x_error, y_error]=l.propagate_error(X_curr, Y_int, 1e-5, minmaxbounds);
                err=y_error;
                x_delta=x_error.*0.1;
                X_curr=X_curr + x_delta;
                
                if abs(x_error)<0.0001
                    %local minimum, can not get better :(
                    break;
                end
                
                
            end
            
            
            X_est = norm2range(X_curr, l.inpRange, l.inpOffset);
            
            [Y_est, conf] = l.apply(X_est);
            Y_err=sqrt(sum((Y_est-Y_des).^2));
            
            
        end
        
  
        
        %batch intrinsic plasticity stuff
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

