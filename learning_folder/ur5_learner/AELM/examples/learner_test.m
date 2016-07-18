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

nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500]; %15000 15625];
%nb_data_total = [14000]; % To first find the right parameters for the ELM (regression and hidden dimension).

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




    
    

%define modalities and dimensionality
modDims = [6 6 1];
%define RBF based learner + parameters
%b: width of activation function as before, this time Gaussian activations / prototypes
%hbdist: distance threshold to insert a new node, lower value more nodes
lspecs = {'class', 'ARBF'; 'hbdist', 1; 'c', [0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05]; 'b', 1; 'regOut', [20 20 20]; 'conv', 1};
% Maybe to be changed to the number of dimension!!!!!

%create learner instance:
cmd = ['l = ' lspecs{1,2} '(modDims, lspecs);'];
eval(cmd);




% Cross validation
nb_fold = 5;
c = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test=zeros(nb_fold,6);
errors_train=zeros(nb_fold,6);
var_errors_test=zeros(nb_fold,6);
var_errors_train=zeros(nb_fold,6);


nr_train_s = c.TrainSize;
nr_test_s = c.TestSize;

nb_train_data = [nb_train_data nr_train_s(1)];


for n=1:nb_fold
	trIdx=find(training(c,n));
	teIdx=find(test(c,n));
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


%% Algorithm testing

%predict one step of the learner
% and feed it back, there are also some functions available to do this and maybe clamp some inputs to get not changed...

temp_error = [];

threshold1 = 0.0001; % To change!
threshold2 = 100000; % To change!




%for each testsample:
for i=1:nr_test_s(n)

    torque_pred=[mean(Y_total(1,:)) mean(Y_total(2,:)) mean(Y_total(3,:)) mean(Y_total(4,:)) mean(Y_total(5,:)) mean(Y_total(6,:))]; %initial guess, some value, maybe just the mean of the value range

    currentval = [samples_test_X(:,i)' torque_pred samples_test_mass(i)];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(torques_next - torque_pred)/6; 

        %move only payload, fix other values:
        torque_pred=torques_next;
        currentval = [samples_test_X(:,i)' torque_pred samples_test_mass(i)];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end

    temp_error = [temp_error abs(torque_pred' - samples_test_Y(:,i))];


end

for i=1:6
    errors_test(n,i) = (sum((temp_error(i,:)).^2,2))/(nr_test_s(n));
    var_errors_test(n,i) = var(temp_error(i,:));
end    

% Evaluation of training
temp_error = [];

for i=1:nr_train_s(n)

    torque_pred=[mean(Y_total(1,:)) mean(Y_total(2,:)) mean(Y_total(3,:)) mean(Y_total(4,:)) mean(Y_total(5,:)) mean(Y_total(6,:))]; %initial guess, some value, maybe just the mean of the value range

    currentval = [samples_train_X(:,i)' torque_pred samples_train_mass(i)];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(torques_next - torque_pred)/6; 

        %move only payload, fix other values:
        torque_pred=torques_next;
        currentval = [samples_train_X(:,i)' torque_pred samples_train_mass(i)];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end

    temp_error = [temp_error abs(torque_pred' - samples_train_Y(:,i))];


end

for i=1:6
    errors_train(n,i) = (sum((temp_error(i,:)).^2,2))/(nr_train_s(n));
    var_errors_train(n,i) = var(temp_error(i,:));
end    
   
end


 temp_train_error = [];
 temp_test_error = [];
 temp_var_train = [];
 temp_var_test = [];
        for jntidx = 1:6
            temp_train_error = [temp_train_error ; mean(errors_train(:,jntidx))];
            temp_test_error = [temp_test_error ; mean(errors_test(:,jntidx))];
            temp_var_test = [temp_var_test ; mean(var_errors_test(:,jntidx))];
            temp_var_train = [temp_var_train ; mean(var_errors_train(:,jntidx))];
        end    

jnt_error_test = [jnt_error_test temp_test_error]
var_test = [var_test temp_var_test];
jnt_error_train = [jnt_error_train temp_train_error];
var_train = [var_train temp_var_train];

% Evaluation of extrapolation

temp_error = [];

%for each testsample:
for i=1:size(Y_total_extrapol,1)

    payload=(5-0.0001)/2; %initial guess, some value, maybe just the mean of the value range

    torque_pred=[mean(Y_total(1,:)) mean(Y_total(2,:)) mean(Y_total(3,:)) mean(Y_total(4,:)) mean(Y_total(5,:)) mean(Y_total(6,:))]; %initial guess, some value, maybe just the mean of the value range
    currentval = [X_total_extrapol(:,i)' torque_pred mass_total_extrapol(i)];

    dist=Inf(1);
    loopcounter=0;
    
    while (dist>threshold1 && loopcounter<threshold2) %as long as payload estimation is moving fastly

        res = l.apply(currentval);
        positions_next = res(1:6);
        torques_next = res(7:12);
        payload_next = res(13);

        %calculate distance for convergence checking:
        dist = norm(torques_next - torque_pred)/6; 

        %move only payload, fix other values:
        torque_pred=torques_next;
        currentval = [X_total_extrapol(:,i)' torque_pred mass_total_extrapol(i)];

        %count loops in case payload estimation does not stop stop update loop
        loopcounter=loopcounter+1;
    end

    temp_error = [temp_error abs(torque_pred' - Y_total_extrapol(:,i))];

end
 

jnt_error_extrapol = [jnt_error_extrapol (sum((temp_error).^2,2))/size(Y_total_extrapol,1)];
var_error_extrapol = [var_error_extrapol var(temp_error)];


end

for jntidx = 1:6
    subplot(2,3,jntidx);
    
plot(nb_train_data(1:end) , jnt_error_test(jntidx,(1:end)));
hold on;
plot(nb_train_data(1:end) , jnt_error_train(jntidx,(1:end)));
plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx,(1:end)));
str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs');
ylabel('Mean Error');
legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
xlabel('Number of inputs');
title(str_title);
end