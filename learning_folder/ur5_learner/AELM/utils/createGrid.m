function [X] = createGrid(dim, min, max, steps, permute);
%CREATEGRID Creates a matrix X of dimension steps^dim x dim
%   X = createGrid(dim, min, max, steps, permute)

if nargin < 5
    permute = false;
end

X = [];

if dim == 1
    X = linspace(min, max, steps)';
elseif dim == 2
    if size(min,1) == 1
        X = zeros(steps*steps, dim);
        x1 = linspace(min, max, steps(1));
        
        cnt = 1;
        for i=1:steps
            for j=1:steps
                X(cnt,:) = [x1(i) x1(j)];
                cnt = cnt + 1;
            end
        end
    else
        X = zeros(steps(1)*steps(2), dim);
        x1 = linspace(min(1), max(1), steps(1));
        x2 = linspace(min(2), max(2), steps(2));
        
        cnt = 1;
        for i=1:steps(1)
            for j=1:steps(2)
                X(cnt,:) = [x1(i) x2(j)];
                cnt = cnt + 1;
            end
        end
    end
elseif dim == 3
    if size(min,1) == 1
        X = zeros(steps*steps*steps, dim);
        x1 = linspace(min, max, steps(1));
        
        cnt = 1;
        for i=1:steps
            for j=1:steps
                for k=1:steps
                    X(cnt,:) = [x1(i) x1(j) x1(k)];
                    cnt = cnt + 1;
                end
            end
        end
    else
        X = zeros(steps(1)*steps(2)*steps(3), dim);
        x1 = linspace(min(1), max(1), steps(1));
        x2 = linspace(min(2), max(2), steps(2));
        x3 = linspace(min(3), max(3), steps(3));
        
        cnt = 1;
        for i=1:steps(1)
            for j=1:steps(2)
                for k=1:steps(3)
                    X(cnt,:) = [x1(i) x2(j) x3(k)];
                    cnt = cnt + 1;
                end
            end
        end
    end
elseif dim == 4
    if size(min,1) == 1
        X = zeros(steps^4, dim);
        x1 = linspace(min, max, steps(1));
        
        cnt = 1;
        for i=1:steps
            for j=1:steps
                for k=1:steps
                    for l=1:steps
                        X(cnt,:) = [x1(i) x1(j) x1(k) x1(l)];
                        cnt = cnt + 1;
                    end
                end
            end
        end
    else
        X = zeros(steps(1)*steps(2)*steps(3)*steps(4), dim);
        x1 = linspace(min(1), max(1), steps(1));
        x2 = linspace(min(2), max(2), steps(2));
        x3 = linspace(min(3), max(3), steps(3));
        x4 = linspace(min(4), max(4), steps(4));
        
        cnt = 1;
        for i=1:steps(1)
            for j=1:steps(2)
                for k=1:steps(3)
                    for l=1:steps(4)
                        X(cnt,:) = [x1(i) x2(j) x3(k) x4(l)];
                        cnt = cnt + 1;
                    end
                end
            end
        end
    end
else
    disp(['createGrid not implemented for dim ' num2str(dim)]);
end

if permute
    X = X(randperm(size(X,1)),:);
end

end

