function [h] = plotMotionMap(X, E, alims, asize, cl, cmds, h)
%PLOTMOTIONMAP Summary of this function goes here
%   Detailed explanation goes here

if nargin < 7
    h = figure;
    axis off;
    hold on;
end

if nargin < 4
    asize = 0.1;
end

if nargin < 5
    cl = '-k';
end

x1 = 1:size(X,2)/2;
x2 = x1(end)+1:size(X,2);

Eoffset = mean(E, 1);
E = E - repmat(Eoffset, size(E,1), 1);
Erange = [min(E, [], 1); max(E, [], 1)];
E = range2norm(E, Erange, [0 0], repmat([asize 1-asize]', 1, 2));

for i=1:size(X,1)
    axes('Position', [E(i,1)-asize/2 E(i,2)-asize/2 asize asize]);
    if iscell(cl)
        if length(cl) == 3
            plot(X(i,x1), X(i,x2), 'LineStyle', cl{1}, 'Color', cl{2}, 'LineWidth', cl{3});
        else
            plot(X(i,x1), X(i,x2), 'LineStyle', cl{i,1}, 'Color', cl{i,2}, 'LineWidth', cl{i,3});
        end
    else
        plot(X(i,x1), X(i,x2), cl);
    end
    
    if nargin > 5
        hold on;
        cmd = cmds(i,:);
        for j=1:length(cmd)
            eval(cmd{j});
        end
    end
    
    axis(alims);
    daspect([1 1 1]);
    axis off;
end

end

