function [Y] = normalizeDiscreteMotion(X, targetLength, yalignment)
%NORMALIZEDISCRETEMOTION Summary of this function goes here
%   Detailed explanation goes here

if nargin < 2
    targetLength = 100;
end

if nargin < 3
    yalignment = 1;
end

% resample
%X = resample(X, targetLength);

% end in origin
X = X - repmat(X(end,:), size(X,1), 1);

s = norm(X(1,:));
X = bsxfun(@rdivide, X, -s * ones(1,size(X,2)));

% rotate to bring start position to [1 0 (0)]
if size(X,2) == 2
    alpha = atan2(X(1,2), X(1,1));
    R = [cos(alpha) sin(alpha); -sin(alpha) cos(alpha)];
    Y = X *  R';
    
    if yalignment
        m = mean(Y,1);
        if m(2) < 0
            Y(:,2) = -Y(:,2);
        end
        %if length(find(Y(:,2) < 0)) > 0.25 * targetLength
        %    Y(:,2) = -Y(:,2);
        %end
    end
elseif size(X,2) == 3
    target_dir = X(1,:);   
    R = align2x(eye(3), target_dir);
    Y = X *  R;

    if yalignment
        m = mean(Y,1);
        xangle = acos(m(2:3) * [1; 0] / norm(m(2:3)));
        if m(3) < 0
            xangle = -xangle;
        end
        Y = Y * rotx(xangle);
    end
else
    warning('not implemented');
end

Y = -Y;

