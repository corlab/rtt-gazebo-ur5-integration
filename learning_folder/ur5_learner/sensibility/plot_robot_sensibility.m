%Read file 

file = '/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/sensibility/complete_data/sensibility_data_mass1.txt';
model_mass = 1;

fileID = fopen(file,'r');

a1 = [];
a2 = [];
mass = [];
trq_err_1 = [];
trq_err_2 = [];
trq1 = [];
trq2 = [];
color_mass = [];

while ~feof(fileID)
	% Loop to read file and add data to matrix.
	fscanf(fileID , [ '{ jnt 1: ']);
    a1 = [a1 ;fscanf(fileID , '%f')];
    fscanf(fileID , [' ; jnt 2: ']);
    a2 = [a2 ;fscanf(fileID , '%f')];
    fscanf(fileID , [' ; mass: ']);
    mass = [mass ;fscanf(fileID , '%f')];
    fscanf(fileID , [' ; jnt1_trq: ']);
    trq1 = [trq1 ;abs(fscanf(fileID , '%f'))];
    fscanf(fileID , [' ; jnt2_trq: ']);
    trq2 = [trq2 ;abs(fscanf(fileID , '%f'))];
    fscanf(fileID , [' ; jnt1_error: ']);
    trq_err_1 = [trq_err_1 ;abs(fscanf(fileID , '%f'))];
    fscanf(fileID , [' ; jnt2_error: ']);
    trq_err_2 = [trq_err_2 ;abs(fscanf(fileID , '%f'))];
    fscanf(fileID , [ ' }' '\n']);
end 

fclose(fileID);


l1 = 0.42500;
l2 = 0.39225;



colormap default;
myColorMap = colormap;
xx = [zeros(size(a1,1),1)];
yy = [zeros(size(a1,1),1)];

xx = [xx l1*cos(-a1)];
yy = [yy l1*sin(-a1)];

xx = [xx xx(:,2)+l2*cos(-a1-a2)];
yy = [yy yy(:,2)+l2*sin(-a1-a2)];

color_mass = myColorMap(floor((mass-model_mass)*63/max(mass-model_mass)+1),:);
color_trq1 = myColorMap(floor((trq1 - min(trq1))*63/(max(trq1)-min(trq1))+1),:);
color_trq2 = myColorMap(floor((trq2 - min(trq2))*63/(max(trq2)-min(trq2))+1),:);
color_trq_err1 = myColorMap(floor((trq_err_1 - min(trq_err_1))*63/(max(trq_err_1) - min(trq_err_1))+1),:);
color_trq_err2 = myColorMap(floor((trq_err_2 - min(trq_err_2))*63/(max(trq_err_2) - min(trq_err_2))+1),:);

xx1= [];
xx0 = [];
yy1 = [];
yy0 = [];
color_mass1 = [];
color_mass0 = [];
color_trq1_1 = [];
color_trq2_1 = [];
color_trq1_0 = [];
color_trq2_0 = [];
color_tr_err1_1 = [];
color_tr_err2_1 = [];
color_tr_err1_0 = [];
color_tr_err2_0 = [];

mass_plot_0 = [];
mass_plot_1 = [];
tr_err1_plot_0 = [];
tr_err2_plot_0 = [];
tr1_plot_0 = [];
tr2_plot_0 = [];
tr_err1_plot_1 = [];
tr_err2_plot_1 = [];
tr1_plot_1 = [];
tr2_plot_1 = [];

% Separation of data for positive and negative angles mapping.
for i=1:size(a2,1)
    if a2(i)>0
        xx1 = [xx1; xx(i,:)];
        yy1 = [yy1; yy(i,:)];
        color_tr_err1_1 = [color_tr_err1_1 ; color_trq_err1(i,:)];
        color_tr_err2_1 = [color_tr_err2_1 ; color_trq_err2(i,:)];
        color_trq1_1 = [color_trq1_1 ; color_trq1(i,:)];
        color_trq2_1 = [color_trq2_1 ; color_trq2(i,:)];
        color_mass1 = [color_mass1; color_mass(i,:)];
        mass_plot_1 = [mass_plot_1 ; (mass(i)-model_mass)];
        tr_err1_plot_1 = [tr_err1_plot_1 ; trq_err_1(i) - min(trq_err_1)];
        tr_err2_plot_1 = [tr_err2_plot_1 ; trq_err_1(i) - min(trq_err_1)];
        tr1_plot_1 = [tr1_plot_1 ; trq1(i) - min(trq1)];
        tr2_plot_1 = [tr2_plot_1 ; trq2(i) - min(trq2)];
    else
        xx0 = [xx0; xx(i,:)];
        yy0 = [yy0; yy(i,:)];
        color_tr_err1_0 = [color_tr_err1_0 ; color_trq_err1(i,:)];
        color_tr_err2_0 = [color_tr_err2_0 ; color_trq_err2(i,:)];
        color_trq1_0 = [color_trq1_0 ; color_trq1(i,:)];
        color_trq2_0 = [color_trq2_0 ; color_trq2(i,:)];
        color_mass0 = [color_mass0; color_mass(i,:)]; 
        mass_plot_0 = [mass_plot_0 ; (mass(i)-model_mass)];
        tr_err1_plot_0 = [tr_err1_plot_0 ; trq_err_1(i) - min(trq_err_1)];
        tr_err2_plot_0 = [tr_err2_plot_0 ; trq_err_1(i) - min(trq_err_1)];
        tr1_plot_0 = [tr1_plot_0 ; trq1(i) - min(trq1)];
        tr2_plot_0 = [tr2_plot_0 ; trq2(i) - min(trq2)];

    end
end   

hold off;
hold on;

str_title=sprintf('Sensitivity as a function of position');
figure_saver = figure('Name',str_title,'NumberTitle','on');

subplot(1,2,1);
hold on;
for i=1:size(xx1,1)
  plot(xx1(i,:),yy1(i,:), 'Color', color_mass1(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(mass-model_mass);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Sensitivity in kg';
set(gca,'color','none')
str_title = sprintf('Sensitivity as a function of joint angle 1 and 2 - positive - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq1,yq1] = meshgrid(min(xx1(:,3)):0.01:max(xx1(:,3)), min(yy1(:,3)):0.01:max(yy1(:,3)));
vq_1 = griddata(xx1(:,3),yy1(:,3),mass_plot_1(:,1),xq1,yq1);
surf(xq1,yq1,vq_1);




subplot(1,2,2);
hold on;
for i=1:size(xx0,1)
  plot(xx0(i,:),yy0(i,:), 'Color', color_mass0(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(mass-model_mass);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Sensitivity in kg';
set(gca,'color','none')
str_title = sprintf('Sensitivity as a function of joint angle 1 and 2 - negative - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq0,yq0] = meshgrid(min(xx0(:,3)):0.01:max(xx0(:,3)), min(yy0(:,3)):0.01:max(yy0(:,3)));
vq_0 = griddata(xx0(:,3),yy0(:,3),mass_plot_0(:,1),xq0,yq0);
surf(xq0,yq0,vq_0);


savefig(figure_saver , 'sens.fig' , 'compact');
close(figure_saver);


str_title=sprintf('Trq 1 and 2 as a function of position');
figure_saver = figure('Name',str_title,'NumberTitle','on');


subplot(2,2,1);
hold on;
for i=1:size(xx1,1)
  plot(xx1(i,:),yy1(i,:), 'Color', color_trq1_1(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq1)-min(trq1);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq1 as a function of joint angle 1 and 2 - positive - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq1,yq1] = meshgrid(min(xx1(:,3)):0.01:max(xx1(:,3)), min(yy1(:,3)):0.01:max(yy1(:,3)));
vq_1 = griddata(xx1(:,3),yy1(:,3),tr1_plot_1,xq1,yq1);
surf(xq1,yq1,vq_1);

subplot(2,2,2);
hold on;
for i=1:size(xx1,1)
  plot(xx1(i,:),yy1(i,:), 'Color', color_trq2_1(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq2)-min(trq2);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq2 as a function of joint angle 1 and 2 - positive - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq1,yq1] = meshgrid(min(xx1(:,3)):0.01:max(xx1(:,3)), min(yy1(:,3)):0.01:max(yy1(:,3)));
vq_1 = griddata(xx1(:,3),yy1(:,3),tr2_plot_1,xq1,yq1);
surf(xq1,yq1,vq_1);

subplot(2,2,3);
hold on;
for i=1:size(xx0,1)
  plot(xx0(i,:),yy0(i,:), 'Color', color_trq1_0(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq1)-min(trq1);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq1 as a function of joint angle 1 and 2 - negative - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq0,yq0] = meshgrid(min(xx0(:,3)):0.01:max(xx0(:,3)), min(yy0(:,3)):0.01:max(yy0(:,3)));
vq_0 = griddata(xx0(:,3),yy0(:,3),tr1_plot_0,xq0,yq0);
surf(xq0,yq0,vq_0);

subplot(2,2,4);
hold on;
for i=1:size(xx0,1)
  plot(xx0(i,:),yy0(i,:), 'Color', color_trq2_0(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq2)-min(trq2);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq2 as a function of joint angle 1 and 2 - negative - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq0,yq0] = meshgrid(min(xx0(:,3)):0.01:max(xx0(:,3)), min(yy0(:,3)):0.01:max(yy0(:,3)));
vq_1 = griddata(xx0(:,3),yy0(:,3),tr2_plot_0,xq0,yq0);
surf(xq0,yq0,vq_0);

savefig(figure_saver , 'torque.fig' , 'compact');
close(figure_saver);




str_title=sprintf('Error Trq 1 and 2 as a function of position');
figure_saver = figure('Name',str_title,'NumberTitle','on');



subplot(2,2,1);
hold on;
for i=1:size(xx1,1)
  plot(xx1(i,:),yy1(i,:), 'Color', color_tr_err1_1(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq_err_1) - min(trq_err_1);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque error';
set(gca,'color','none')
str_title = sprintf('Trq1 error as a function of joint angle 1 and 2 - positive - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq1,yq1] = meshgrid(min(xx1(:,3)):0.01:max(xx1(:,3)), min(yy1(:,3)):0.01:max(yy1(:,3)));
vq_1 = griddata(xx1(:,3),yy1(:,3),tr_err1_plot_1,xq1,yq1);
surf(xq1,yq1,vq_1);

subplot(2,2,2);
hold on;
for i=1:size(xx1,1)
  plot(xx1(i,:),yy1(i,:), 'Color', color_tr_err2_1(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq_err_2) - min(trq_err_2);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque error';
set(gca,'color','none')
str_title = sprintf('Trq2 error as a function of joint angle 1 and 2 - positive - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq1,yq1] = meshgrid(min(xx1(:,3)):0.01:max(xx1(:,3)), min(yy1(:,3)):0.01:max(yy1(:,3)));
vq_1 = griddata(xx1(:,3),yy1(:,3),tr_err2_plot_1,xq1,yq1);
surf(xq1,yq1,vq_1);

subplot(2,2,3);
hold on;
for i=1:size(xx0,1)
  plot(xx0(i,:),yy0(i,:), 'Color', color_tr_err1_0(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq_err_1) - min(trq_err_1);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq1 error as a function of joint angle 1 and 2 - negative - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq0,yq0] = meshgrid(min(xx0(:,3)):0.01:max(xx0(:,3)), min(yy0(:,3)):0.01:max(yy0(:,3)));
vq_0 = griddata(xx0(:,3),yy0(:,3),tr_err1_plot_0,xq0,yq0);
surf(xq0,yq0,vq_0);

subplot(2,2,4);
hold on;
for i=1:size(xx0,1)
  plot(xx0(i,:),yy0(i,:), 'Color', color_tr_err2_0(i,:) , 'LineWidth',2);
end
h = colorbar;
h.Ticks=[0,0.5,1];
max_str = max(trq_err_2) - min(trq_err_2);
mid_str = (max_str)/2;
h.TickLabels={'0',mid_str,max_str};
h.Label.String = 'Torque';
set(gca,'color','none')
str_title = sprintf('Trq2 error as a function of joint angle 1 and 2 - negative - model: %d kg',model_mass);
title(str_title);
xlabel('x');
ylabel('y');
axis equal;
[xq0,yq0] = meshgrid(min(xx0(:,3)):0.01:max(xx0(:,3)), min(yy0(:,3)):0.01:max(yy0(:,3)));
vq_0 = griddata(xx0(:,3),yy0(:,3),tr_err2_plot_0,xq0,yq0);
surf(xq0,yq0,vq_0);

savefig(figure_saver , 'torque_error.fig' , 'compact');
close(figure_saver);