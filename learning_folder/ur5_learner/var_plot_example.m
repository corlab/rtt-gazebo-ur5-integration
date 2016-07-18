%variance test plot:

nrsamples = 200;


x_data = -2*pi:0.1:2*pi;
y_data = repmat(sin(x_data),nrsamples,1);
y_data = y_data + repmat(sin(0.5*x_data),nrsamples,1).*(rand(size(y_data,1),size(y_data,2))-0.5);

%dplot data
figure(1)
plot(y_data')

%calculate mean and variance:
mean_data = mean(y_data);
var_data = var(y_data);
stddev_data=sqrt(var_data);



%plot line variance:
fig_mean = figure(2)



lineadata_var_x=[x_data flip(x_data)];
lineadata_std1_y=[mean_data+stddev_data flip(mean_data-stddev_data)];
lineadata_std2_y=[mean_data+2.*stddev_data flip(mean_data-2.*stddev_data)];

p2 = fill(lineadata_var_x,lineadata_std2_y,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
hold on;
p1 = fill(lineadata_var_x,lineadata_std1_y,[0.82 0.82 0.82], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]

hold on;
grid on;
g1 = plot(x_data, mean_data,'k');
legend_handle = legend([g1, p1, p2], {'Mean Reward', 'Std. deviation 68.3% of data', '2xStd. deviation 95.4% of data'}, 'Location','southeast');
ylabel('Mean Values');
xlabel('Input');
set(gca,'Layer','top')
xlim([x_data(1),x_data(end)])
set(fig_mean, 'Position', [300 300 700 400])

