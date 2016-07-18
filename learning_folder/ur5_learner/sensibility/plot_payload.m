figure()

subplot(1,2,2)
xx1 = [0.0001 0.7 1 2.3 3 4.2 5];
yy1 = [1 1 1 1 1 1 1];

plot(xx1,yy1, 'Color', 'red');
hold on;
xx11 = [0.7 1];
xx12 = [2.3 3];
xx13 = [4.2 5];
yy11 = [1 1];
plot(xx11,yy11, 'Color', 'blue');
plot(xx12,yy11, 'Color', 'blue');
plot(xx13,yy11, 'Color', 'blue');
vert1 = [0.5 0.5];
vert2 = [2 2];
vert3 = [4 4];
yvert = [0.5 1.5];
plot(vert1 , yvert , 'Color', 'black');
plot(vert2,yvert, 'Color', 'black');
plot(vert3,yvert, 'Color', 'black');
legend('Payload guessing for second position');
ax = gca;
ax.XTick = [0.0001 0.7 1 2.3 3 4.2 5];
ax.YTick = [];
ax2 = copyobj(gca, gcf);                             %// Create a copy the axes
set(ax2, 'XTick', [0.0001,1,3, 5], 'XColor', 'r', 'Color', 'none')
ax3 = copyobj(gca, gcf);                             %// Create another copy
set(ax3, 'XTick', [], 'Color', 'none') 
xlabel('payload');



subplot(1,2,1)
xx2 = [0.0001 0.5 1 2 3 4 5];
yy2 = [1 1 1 1 1 1 1];
plot(xx2,yy2, 'Color', 'red');
hold on;
xx11 = [0.5 1];
xx12 = [2 3];
xx13 = [4 5];
yy11 = [1 1];
plot(xx11,yy11, 'Color', 'blue');
plot(xx12,yy11, 'Color', 'blue');
plot(xx13,yy11, 'Color', 'blue');
vert1 = [0.5 0.5];
vert2 = [2 2];
vert3 = [4 4];
yvert = [0.5 1.5];
plot(vert1 , yvert, 'Color', 'black');
plot(vert2,yvert, 'Color', 'black');
plot(vert3,yvert, 'Color', 'black');
legend('Payload guessing for first position');
ax = gca;
ax.XTick = [0.0001 0.5 1 2 3 4 5];
ax.YTick = [];
ax2 = copyobj(gca, gcf);                             %// Create a copy the axes
set(ax2, 'XTick', [0.0001,1,3, 5], 'XColor', 'r', 'Color', 'none')
ax3 = copyobj(gca, gcf);                             %// Create another copy
set(ax3, 'XTick', [], 'Color', 'none') 
xlabel('payload');