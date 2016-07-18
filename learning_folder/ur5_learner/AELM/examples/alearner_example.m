
close all;
clear all;

interactiveTesting = 1;


%% example data
noise = 0.03;
numSamples = 100;
X = linspace(-1, 1, numSamples)';
Y = 4 * X + 0.5;
Y = 1./(1+exp(-Y));
Y = normalize(Y);
Y = Y + noise * randn(numSamples, 1);
X = [X Y];

if 1
%% a 1d manifold with branch
noise = 0.03;
%linear part
x = [-2:0.01:0]';
y = x;
%first branch
x1 = [0:0.01:3]';
y1 = sqrt(x1);
%second branch
x2 = [0:0.01:2]';
y2 = -sqrt(x2);
%collect all parts
X = [x; x1; x2];
Y = [y; y1; y2];
%normalize
X = normalize(X);
Y = normalize(Y);
%add noise
Y = Y + noise * randn(size(Y));
X = [X Y];
end

G = createGrid(2, [-1.2; -1.2], [1.2; 1.2], 15 * ones(2,1));


%% learner
modDims = [1 1];
%lspecs = {'class', 'AELM'; 'hidDim', 100; 'g', [1/2 1/2]; 'regInp', [1e-1 1e-1]; 'regOut', 1e-2; 'conv', 1};
lspecs = {'class', 'ARBF'; 'hbdist', 0.1; 'c', [1 1]; 'b', 1; 'regOut', [1e-3 1e-3]; 'conv', 1};

cmd = ['l = ' lspecs{1,2} '(modDims, lspecs);'];
eval(cmd);

l.init(X);
l.train(X,X);

%% evaluation
if ~interactiveTesting
%l.conv = 0;
%l.g = [0.5 0.25 0.25];
l.setDrivenMods(1);
Yhat = l.apply(X);
E = sqrt(sum((X(:,2) - Yhat(:,2)).^2, 2));
rmse1 = mean(E)
stdrmse1 = std(E)

%l.conv = 0;
%l.g = [0.25 0.5 0.25];
l.setDrivenMods(2);
Xhat = l.apply(X);
E = sqrt(sum((X(:,1) - Xhat(:,1)).^2, 2));
rmse2 = mean(E)
stdrmse2 = std(E)

% plot
figure; subplot(1,2,1);
hold on; box on; grid on;
plot(X(:,1), X(:,2), '.k');
plot(X(:,1), Yhat(:,2), '-b', 'LineWidth', 2);
xlabel('x'); ylabel('y');
daspect([1 1 1]);
axis([-1.2 1.2 -1.2 1.2]);

subplot(1,2,2); %figure; 
hold on; box on; grid on;
plot(X(:,2), X(:,1), '.k');
plot(X(:,2), Xhat(:,1), '-b', 'LineWidth', 2);
xlabel('y'); ylabel('x');
daspect([1 1 1]);
axis([-1.2 1.2 -1.2 1.2]);


else
%%%%%%%%%%%%%%%%%% interactive testing %%%%%%%%%%%%%%%%%%%%%%%
x = X(1,1);
y = X(1,2);
xy = [x y];
l.setDrivenMods([1 2]);

close all;
h = figure;
rotate3d on;
set(gcf, 'SelectionType', 'open');
stop = 0;
alt = 0;
while ~stop
    point = get(gca, 'CurrentPoint');
    button = get(gcf,'SelectionType');
    target = [point(1,1) point(1,2)];
    
    if strcmp(button, 'extend')
        x = target(1);
        y = target(2);
        if target(1) < -1.2
            stop = 1;
        end
    elseif strcmp(get(rotate3d(h),'Enable'), 'on')
        x = target(1);
    end
    set(gcf, 'SelectionType', 'open');

    clf(h);
    hold on; box on; grid on;
    daspect([1 1 1]);
    axis([-1.2 1.2 -1.2 1.2]);
    
    plot(X(:,1), X(:,2), '.k');
    plot(x, y, '*m');
    plot(target(1), target(2), '*r');
    
    
    [xyhat] = l.apply([x y]);
    %[xyhat] = l.apply([target(1), target(2)]);
    hs=l.calcHiddenStates([target(1), target(2)])
    
    %[d,n]=l.nearestNeighbors([target(1), target(2)], 3);
    %d
    %n
    points = l.Winp;
    plot(points(:,1), points(:,2), '*r');
    x = xyhat(1);
    y = xyhat(2);
    
    
    
    Ghat = l.apply(G);
    arrow(G, Ghat, 'Length', 5);
    
    pause(0.05);
end

close(h);
end





