function [  ] = plotData( samples_X , samples_Y , massidx , samples_test_X , samples_test_Y , elm_learner )
%PLOT_DATA Summary of this function goes here
%   Detailed explanation goes here



% This part doesn't change in function of the ELM parameters but only on the
% input data. It doesn't need to be computed at each simulation.
str_title=sprintf('Torques as function of angles');
figure_saver2 = figure('Name',str_title,'NumberTitle','on');

subplot(2,3,1);
str_title=sprintf('Torque as function of angle 0');
plot(samples_X(1,: , massidx),samples_Y(1,: , massidx) , '*');
title(str_title);
subplot(2,3,2);
str_title=sprintf('Torque as function of angle 1');
plot(samples_X(2,:, massidx),samples_Y(2,:, massidx) , '*');
title(str_title);
subplot(2,3,3);
str_title=sprintf('Torque as function of angle 2');
plot(samples_X(3,:, massidx),samples_Y(3,:, massidx) , '*');
title(str_title);
subplot(2,3,4);
str_title=sprintf('Torque as function of angle 3');
plot(samples_X(4,:, massidx),samples_Y(4,:, massidx) , '*');
title(str_title);
subplot(2,3,5);
str_title=sprintf('Torque as function of angle 4');
plot(samples_X(5,:, massidx),samples_Y(5,:, massidx) , '*');
title(str_title);
subplot(2,3,6);
str_title=sprintf('Torque as function of angle 5');
plot(samples_X(6,:, massidx),samples_Y(6,:, massidx) , '*');
title(str_title);

savefig(figure_saver2 , 'torque_angles.fig' , 'compact');
close(figure_saver2);


str_title=sprintf('Torques as function of 3 angles');
figure_saver3 = figure('Name',str_title,'NumberTitle','on');
subplot(3,1,1);
str_title=sprintf('Torque 0 as function of angles 0, 1, 2');
scatter3(samples_X(1,:,1) , samples_X(2,:,1) , samples_X(3,:,1) ,20, samples_Y(1,:,1) , 'filled');
xlabel('angle 0')
ylabel('angle 1')
zlabel('angle 2')
cb = colorbar; 
cb.Label.String = 'torque 0';
title(str_title);
subplot(3,1,2);
str_title=sprintf('Torque 1 as function of angles 0, 1, 2');
scatter3(samples_X(1,:,1) , samples_X(2,:,1) , samples_X(3,:,1) ,20, samples_Y(2,:,1) , 'filled');
xlabel('angle 0')
ylabel('angle 1')
zlabel('angle 2')
cb = colorbar; 
cb.Label.String = 'torque 1';
title(str_title);
subplot(3,1,3);
str_title=sprintf('Torque 2 as function of angles 0, 1, 2');
scatter3(samples_X(1,:,1) , samples_X(2,:,1) , samples_X(3,:,1) ,20, samples_Y(3,:,1) , 'filled');
xlabel('angle 0')
ylabel('angle 1')
zlabel('angle 2')
cb = colorbar; 
cb.Label.String = 'torque 2';
title(str_title);
savefig(figure_saver3 , 'torque_3angles.fig' , 'compact');
close(figure_saver3);






figure();

subplot(1,2,1);
dataa  = samples_test_X(2:3,:);
datab  = samples_test_Y(2:3,:);
plotdata = elm_learner.apply([zeros(1,size(dataa,2)); dataa; zeros(3,size(dataa,2))]');
[xq,yq] = meshgrid(-2.5:0.2:0.2, -2.5:.2:2.5);
vq = griddata(dataa(1,:),dataa(2,:),plotdata(:,2),xq,yq);
mesh(xq,yq,vq);
hold on
plot3(dataa(1,:), dataa(2,:), datab(1,:), 'x');

subplot(1,2,2);
[xq,yq] = meshgrid(-2.5:0.2:0.2, -2.5:.2:2.5);
vq = griddata(dataa(1,:),dataa(2,:),plotdata(:,3),xq,yq);
mesh(xq,yq,vq);
hold on
plot3(dataa(1,:), dataa(2,:), datab(2,:), 'x');

end

