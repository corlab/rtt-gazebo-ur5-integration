%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%          Copyright (c) 2013 R. F. Reinhart, CoR-Lab                 %%%
%%%          Bielefeld University, Germany, http://cor-lab.de           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [X,Q] = fkineNDOFArm(Q, l)

    n = size(Q,2);
    numSamples = size(Q,1);

    X = zeros(numSamples,2);

    for s=1:numSamples
        for i=1:n
            tmp = sum(Q(s,1:i));
            X(s,:) = X(s,:) + [cos(tmp) sin(tmp)];
        end
    end

    if nargin < 2
        X = (1/n) .* X;
    else
%         X = l .* (1/n) .* X;
        X = l .* X;
    end

end