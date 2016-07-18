clear all;
close all;
    upath = '../';
    
    addpath([upath filesep 'utils']);
    addpath([upath filesep 'metric']);
    addpath([upath filesep 'learner']);
    addpath([upath filesep 'examples']);

%% Data preparation
[samples_X_total  , samples_Y_total , nr_samples_total] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt');
[samples_X_extrapol , samples_Y_extrapol , nr_samples_extrapol] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_heavy_mass.txt');

X_total = samples_X_total(1:6,:);
mass_total = samples_X_total(7,:);
Y_total = samples_Y_total;

X_total_extrapol = samples_X_extrapol(1:6,:);
mass_total_extrapol = samples_X_extrapol(7,:);
Y_total_extrapol = samples_Y_extrapol;


% -------------- Change number of data for the training.

%nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];
nb_data_total = [15625]; % To first find the right parameters for the ELM (regression and hidden dimension).

nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_error_extrapol = [];
var_train = [];
var_test = [];
var_error_extrapol = [];

X = [];
Y = [];
mass = [];

for id_nb_data = nb_data_total
    operation = [ones(1,id_nb_data) zeros(1,nr_samples_total - id_nb_data) ];  %Fill the vector with 0 and 1
    operation = operation(randperm(nr_samples_total ));  %Randomly reorder it
    nb_column = 0;
    for id = operation
        nb_column = nb_column + 1;
        if id == 1 
            X = cat(2,X, X_total(:,nb_column));
            Y = cat(2,Y, Y_total(:,nb_column));
            mass = cat(2,mass,mass_total(nb_column));
        end    
    end    




    
% First tests
%b=[ 1e-1 1 2 5 7 10 15 20 25 30];
%c=[1e-1 1 2 5 7 10 15];   
%b=[ 1e-1 1 5 10 20 30];
%c=[1e-1 1 2 5 10 15];   



%More precise
%b = [0.5 1 1.5];
%c = [1.5 2 2.5 3 3.5];

b = [0 0.25 0.5 1];
c = [2.5 3 3.5 4 4.5];

resultmatrix_test=zeros(size(b,2), size(c,2));
resultmatrix_var_test=zeros(size(b,2),size(c,2));
resultmatrix_train=zeros(size(b,2), size(c,2));
resultmatrix_var_train=zeros(size(b,2),size(c,2));
hd_size=zeros(size(b,2), size(c,2));
    

nb_config = 0;

idx_b = 0;
idx_c = 0;

for b_index=b
	idx_b = idx_b + 1;
	idx_c = 0;
	for c_index=c   
		idx_c = idx_c+1;
        c_vector = c_index*ones(1,13);
        
        nb_config = nb_config + 1 ;
%define modalities and dimensionality
modDims = [6 6 1];
%define RBF based learner + parameters
%b: width of activation function as before, this time Gaussian activations / prototypes
%hbdist: distance threshold to insert a new node, lower value more nodes
%lspecs = {'class', 'ARBF'; 'hbdist', 0.06; 'c', [1 1 1 1 1 1 1 1 1 1 1 1 1]; 'b', 1; 'regOut', [10 10 10]; 'conv', 1};


% First 2 : hbdist = 1.5
lspecs = {'class', 'ARBF'; 'hbdist', 1.5; 'c', c_vector; 'b', b_index; 'regOut', [0.005 0.005 0.005]; 'conv', 0};

%create learner instance:
cmd = ['l = ' lspecs{1,2} '(modDims, lspecs);'];

% Cross validation
nb_fold = 5;
cpart = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test=zeros(nb_fold,1);
errors_train=zeros(nb_fold,1);
var_errors_test=zeros(nb_fold,1);
var_errors_train=zeros(nb_fold,1);

nr_train_s = cpart.TrainSize;
nr_test_s = cpart.TestSize;

nb_train_data = [nb_train_data nr_train_s(1)];


for n=1:nb_fold
	trIdx=find(training(cpart,n));
	teIdx=find(test(cpart,n));
	nr_train_samples = nr_train_s(n);
	nr_test_samples = nr_test_s(n);
	samples_train_X = [];
	samples_train_Y = [];
    samples_train_mass = [];
	samples_test_X = [];
	samples_test_Y = [];
    samples_test_mass = [];

% Random data chosen to train
	for idx = trIdx
		samples_train_X = [samples_train_X X(: , idx )];
		samples_train_Y = [samples_train_Y Y(: , idx )];
        samples_train_mass = [samples_train_mass mass(idx)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X = [samples_test_X X(: , idx )];
		samples_test_Y = [samples_test_Y Y(: , idx )];
        samples_test_mass = [samples_test_mass mass(idx)];
    end	

    X_train = [samples_train_X; samples_train_Y ;samples_train_mass];

%% Algorithm training

eval(cmd);

%init and train as before
%But! this time you train the identity! so input X -> X ( x contains all modalities [torques positions payload])
l.init(X_train');
l.train(X_train', X_train');
l.setDrivenMods( [1 2 3]);


%% Algorithm testing

%predict one step of the learner
% and feed it back, there are also some functions available to do this and maybe clamp some inputs to get not changed...

temp_error = [];

threshold1 = 0.0001; % To change!
threshold2 = 5000; % To change!




%for each testsample:
for i=1:nr_test_s(n)

    payload=(5-0.0001)/2; %initial guess, some value, maybe just the mean of the value range
    currentval = [samples_test_X(:,i)' samples_test_Y(:,i)' payload];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(payload_next - payload);

        %move only payload, fix other values:
        payload=payload_next;
        currentval = [samples_test_X(:,i)' samples_test_Y(:,i)' payload];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end
  %  [dist loopcounter]
    

    temp_error = [temp_error abs(payload - samples_test_mass(i))];


end

errors_test(n) = (sum((temp_error).^2,2))/(nr_test_s(n));
var_errors_test(n) = var(temp_error);
    

% Evaluation of training
temp_error = [];

for i=1:nr_train_s(n)

    payload=(5-0.0001)/2; %initial guess, some value, maybe just the mean of the value range

    currentval = [samples_train_X(:,i)' samples_train_Y(:,i)' payload];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(payload_next - payload); 

        %move only payload, fix other values:
        payload=payload_next;
        currentval = [samples_train_X(:,i)' samples_train_Y(:,i)' payload];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end

    temp_error = [temp_error abs(payload - samples_train_mass(i))];

end

    errors_train(n) = (sum((temp_error).^2,2))/(nr_train_s(n));
    var_errors_train(n) = var(temp_error);

   
end



resultmatrix_test(idx_b,idx_c)=mean(errors_test);
resultmatrix_var_test(idx_b,idx_c)=mean(var_errors_test);
resultmatrix_train(idx_b,idx_c)=mean(errors_train);
resultmatrix_var_train(idx_b,idx_c)=mean(var_errors_train);
jnt_error_test = [jnt_error_test mean(errors_test)]
var_test = [var_test mean(var_errors_test)];
jnt_error_train = [jnt_error_train mean(errors_train)];
var_train = [var_train mean(var_errors_train)];
hd_size(idx_b,idx_c) = l.hidDim

disp(['nb_config ' num2str(nb_config) ' done']);

% Evaluation of extrapolation
%{
temp_error = [];

%for each testsample:
for i=1:size(mass_total_extrapol,1)

    payload=(5-0.0001)/2; %initial guess, some value, maybe just the mean of the value range

    currentval = [X_total_extrapol(:,i)' Y_total_extrapol(:,i)' payload];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(payload_next - payload); 

        %move only payload, fix other values:
        payload=payload_next;
        currentval = [X_total_extrapol(:,i)' Y_total_extrapol(:,i)' payload];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end

    temp_error = [temp_error abs(payload - mass_total_extrapol(i))];

end

jnt_error_extrapol = [jnt_error_extrapol (sum((temp_error).^2,2))/size(mass_total_extrapol,1)];
var_error_extrapol = [var_error_extrapol var(temp_error)];
%}
   end 
end



    str_title=sprintf('training and testing error');

	figure_saver(1) = figure('Name',str_title,'NumberTitle','on');
	rotate3d on

 
	subplot(2,2,1);
	surf(resultmatrix_test)
	title('Mean Error test')
    xlabel('c')
    ylabel('b')
	rotate3d;

	subplot(2,2,3);
	surf(resultmatrix_var_test)
	title('Variance Error test')
    xlabel('c')
    ylabel('b')
	rotate3d;

	subplot(2,2,2);
	surf(resultmatrix_train)
	title('Mean Error train')
    xlabel('c')
    ylabel('b')
	rotate3d;

	subplot(2,2,4);
	surf(resultmatrix_var_train)
	title('Variance Error train')
    xlabel('c')
    ylabel('b')
	rotate3d;
    savefig(figure_saver , 'results.fig' , 'compact');
    close(figure_saver);

end

%{
plot(nb_train_data(1:end) , jnt_error_test((1:end)));
hold on;
plot(nb_train_data(1:end) , jnt_error_train((1:end)));
%plot(nb_train_data(1:end) , jnt_error_extrapol((1:end)));
str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs');
ylabel('Mean Error');
legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
xlabel('Number of inputs');
title(str_title);
%}