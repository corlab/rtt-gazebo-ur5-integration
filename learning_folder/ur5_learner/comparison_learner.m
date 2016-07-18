clear all;
close all;

%restoredefaultpath;
addpath('../learner/algorithms');
addpath('../learner/metric');
addpath('../learner/utils');

nr_samples = [];

range=[-2*pi,2*pi];


%% -------------  Preparing data set --------------------------------------------------

%fileID = fopen('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/test_data_no_mass_set1.txt','r');

% Data set : 
samples_train_X = [];
samples_train_Y = [];

samples_test_X = [];   
samples_test_Y = [];

samples_X_total = [];
samples_Y_total = [];



[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass0.txt');

samples_X_total = cat(3, samples_X_total ,samples_X_temp);
samples_Y_total = cat(3,samples_Y_total ,samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];



[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass1.txt');
samples_X_total = cat(3,samples_X_total, samples_X_temp);
samples_Y_total = cat(3,samples_Y_total ,samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];
[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass5.txt');
samples_X_total = cat(3,samples_X_total, samples_X_temp);
samples_Y_total = cat(3,samples_Y_total, samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];
[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass3.txt');
samples_X_total = cat(3,samples_X_total, samples_X_temp);
samples_Y_total = cat(3,samples_Y_total, samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];


% Evaluation for different payloads


% Payloads from 0 to 5 kg and payloads from 5 to 10 kg.
[extra_test_X_1 , extra_test_Y_1 , nr_samples_1] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt'); 
[extra_test_X_2 , extra_test_Y_2 , nr_samples_2] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_heavy_mass.txt');


% --------------------------------------------------------------------------------------------------------




% -------------- Change number of data for the training.

nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];
%nb_data_total = [15625];


nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_error_extrapol_1 = [];
jnt_error_extrapol_2 = [];
var_train = [];
var_test = [];
var_extrapol_1 = [];
var_extrapol_2 = [];
var_mean_1 = [];
var_mean_2 = [];



for id_nb_data = nb_data_total

samples_X = [];
samples_Y = [];
operation = [ones(1,id_nb_data) zeros(1,nr_samples_temp - id_nb_data) ];  %Fill the vector with 0 and 1
operation = operation(randperm(nr_samples_temp ));  %Randomly reorder it
nb_column = 0;
for id = operation
   nb_column = nb_column + 1;
   if id == 1 
       samples_X = cat(2,samples_X, samples_X_total(:,nb_column , :));
       samples_Y = cat(2,samples_Y, samples_Y_total(:,nb_column , :));
   end    
end    

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------

%% ------------- Parameters optimization -----------------------------------------

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------


hiddim = 60;
reg = 1e-8;
bipidx = 0;
fw = 1;

disp(['Config: BIP: ' num2str(bipidx)  ' fw: ' num2str(fw) ' reg: ' num2str(reg) ' hiddim: ' num2str(hiddim)])

test_error = zeros(10,6);
test_var = zeros(10,6);
test_error_extrapol = zeros(10,6);
test_var_extrapol = zeros(10,6);
% --------------------------------------------------------------------------------------------------------

% ------------- Multiple testing -----------------------------------------

% --------------------------------------------------------------------------------------------------------

for i=1:10



%elm:
%BIP - Batch intrinsic plasticity (extension for tuning of parameters)
%reg - regularization for linear output learning
%hidDim - hidden layer size (random layer)
%
elm_learner=ELM(6,6,{'BIP',0;'reg', 1e-3; 'hidDim', 500});


% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_fold = 5;
c = cvpartition(id_nb_data,'KFold' , nb_fold);


nr_train_s = c.TrainSize;
nr_test_s = c.TestSize;

if i== 1
    nb_train_data = [nb_train_data nr_train_s(1)];
end

n=1;
trIdx=find(training(c,n));
	teIdx=find(test(c,n));
	nr_train_samples = nr_train_s(n);
	nr_test_samples = nr_test_s(n);
	samples_train_X_0 = [];
	samples_train_Y_0 = [];
	samples_test_X_0 = [];
	samples_test_Y_0 = [];
	samples_train_X_1 = [];
	samples_train_Y_1 = [];
	samples_test_X_1 = [];
	samples_test_Y_1 = [];	
    samples_train_X_3 = [];
	samples_train_Y_3 = [];
	samples_test_X_3 = [];
	samples_test_Y_3 = [];	
    samples_train_X_5 = [];
	samples_train_Y_5 = [];
	samples_test_X_5 = [];
	samples_test_Y_5 = [];

    % Data prepared for each payload
    
% Random data chosen to train
	for idx = trIdx
		samples_train_X_0 = [samples_train_X_0 samples_X(: , idx , 1)];
		samples_train_Y_0 = [samples_train_Y_0 samples_Y(: , idx , 1)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X_0 = [samples_test_X_0 samples_X(: , idx , 1)];
		samples_test_Y_0 = [samples_test_Y_0 samples_Y(: , idx , 1)];
	end	
% Random data chosen to train
	for idx = trIdx
		samples_train_X_1 = [samples_train_X_1 samples_X(: , idx , 2)];
		samples_train_Y_1 = [samples_train_Y_1 samples_Y(: , idx , 2)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X_1 = [samples_test_X_1 samples_X(: , idx , 2)];
		samples_test_Y_1 = [samples_test_Y_1 samples_Y(: , idx , 2)];
    end	
    % Random data chosen to train
	for idx = trIdx
		samples_train_X_3 = [samples_train_X_3 samples_X(: , idx , 4)];
		samples_train_Y_3 = [samples_train_Y_3 samples_Y(: , idx , 4)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X_3 = [samples_test_X_3 samples_X(: , idx , 4)];
		samples_test_Y_3 = [samples_test_Y_3 samples_Y(: , idx , 4)];
    end	
    % Random data chosen to train
	for idx = trIdx
		samples_train_X_5 = [samples_train_X_5 samples_X(: , idx , 3)];
		samples_train_Y_5 = [samples_train_Y_5 samples_Y(: , idx , 3)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X_5 = [samples_test_X_5 samples_X(: , idx , 3)];
		samples_test_Y_5 = [samples_test_Y_5 samples_Y(: , idx , 3)];
	end	

% -----------------------------------------------------------------------------------------------------



% ------------- Training -----------------------------------------------------------------------------



	if (n == 1)
		elm_learner.a=fw.*elm_learner.a;
%scale inputs to range etc:
%elm_learner.b sind die centers der fermi funktionen - alle random
%elm_learner.a 'width' of fermi function, default: 1
%the larger 'a' gets, the 'smaller' the fermi function


%elm_learner.a=0.01.*elm_learner.a; % -> extremely wide fermi function
%almost linear

%elm_learner.a=5.*elm_learner.a; % -> smaller sigmoid in function space
    end

    % Model
	elm_learner.init([samples_train_X_0 samples_train_X_1 samples_train_X_3 samples_train_X_5]' , [samples_train_Y_0 samples_train_Y_1 samples_train_Y_3 samples_train_Y_5]');
    
    elm_learner_0 = elm_learner.copy();
    elm_learner_1 = elm_learner.copy();
    elm_learner_3 = elm_learner.copy();
    elm_learner_5 = elm_learner.copy();   
    
%train the stuff
	elm_learner_0.train(samples_train_X_0', samples_train_Y_0');
   	elm_learner_1.train(samples_train_X_1', samples_train_Y_1');
	elm_learner_3.train(samples_train_X_3', samples_train_Y_3');
	elm_learner_5.train(samples_train_X_5', samples_train_Y_5');


    
% ELM for ELM    
    elm_on_elm=ELM(1, size(reshape(elm_learner.wOut , [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) , 1),{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});
    big_elm_X = [0.0001 1 3 5];
    big_elm_Y = [reshape(elm_learner_0.wOut, [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) reshape(elm_learner_1.wOut, [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) reshape(elm_learner_3.wOut, [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) reshape(elm_learner_5.wOut, [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) ];
    elm_on_elm.init(big_elm_X' , big_elm_Y');
    elm_on_elm.train(big_elm_X' , big_elm_Y');
    
% ---------------------------------------------------------------------------------------------------




% ------------- Evaluation - error calculation ---------------------------------------------------------

  

            %% Evaluation for different payloads

%%%%%%%%%%%% Error computation on extrapolated data (payload from 5kg to 10kg)
%%%%%%%%%% Done once parameters are found regularization and hidden dimension).

    extra_data_error = [];
    extra_data_error_2 = [];

    for idx = 1:size(extra_test_X_1 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(extra_test_X_1(7 , idx )) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(extra_test_X_1(1:6 , idx )');
        extra_data_error = [extra_data_error (elm_result - extra_test_Y_1(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error(i,jntidx)=(sum((extra_data_error(jntidx,:)).^2,2))/nr_samples_1;
        test_var(i,jntidx)=var(extra_data_error(jntidx,:).^2);
    end
    
    for idx = 1:size(extra_test_X_2 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(extra_test_X_2(7 , idx )) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(extra_test_X_2(1:6 , idx )');
        extra_data_error_2 = [extra_data_error_2 (elm_result - extra_test_Y_2(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error_extrapol(i,jntidx)=(sum((extra_data_error_2(jntidx,:)).^2,2))/nr_samples_2;
        test_var_extrapol(i,jntidx)=var(extra_data_error_2(jntidx,:).^2);
    end
end % end of i - loop for testing


    temp_jnt_err = [];
    temp_jnt_var = [];
    temp_jnt_err_extrapol = [];
    temp_jnt_var_extrapol = [];
    var_err = [];
    var_extrapol = [];
    for jntidx = 1:6
        temp_jnt_err = [temp_jnt_err mean(test_error(:,jntidx))];
        temp_jnt_var = [temp_jnt_var mean(test_var(:,jntidx))];
        temp_jnt_err_extrapol = [temp_jnt_err_extrapol mean(test_error_extrapol(:,jntidx))];
        temp_jnt_var_extrapol = [temp_jnt_var_extrapol mean(test_var_extrapol(:,jntidx))];
        var_err = [var_err var(test_error(:,jntidx))];
        var_extrapol = [var_extrapol var(test_error_extrapol(:,jntidx))];
    end    

    
    jnt_error_extrapol_1 = [jnt_error_extrapol_1 temp_jnt_err'];
    jnt_error_extrapol_2 = [jnt_error_extrapol_2 temp_jnt_err_extrapol'];
    var_extrapol_1 = [var_extrapol_1 temp_jnt_var'];
    var_extrapol_2 = [var_extrapol_2 temp_jnt_var_extrapol'];
    var_mean_1 = [var_mean_1 var_err'];
    var_mean_2 = [var_mean_2 var_extrapol'];
    
end





big_jnt_error_extrapol_1 = jnt_error_extrapol_1;
big_jnt_error_extrapol_2 = jnt_error_extrapol_2;
big_var_extrapol_1 = var_extrapol_1;
big_var_extrapol_2 = var_extrapol_2;
big_var_mean_extrapol_1 = var_mean_1;
big_var_mean_extrapol_2 = var_mean_2;
big_nb_train_data = nb_train_data;
% --------------------------------------------------------------------------------------------------




%% Simple learner

nr_samples = [];

range=[-2*pi,2*pi];


% -------------  Preparing data set --------------------------------------------------


% Data set : 
samples_train_X = [];
samples_train_Y = [];

samples_test_X = [];   
samples_test_Y = [];

samples_X_total = [];
samples_Y_total = [];



[samples_X_total  , samples_Y_total , nr_samples_total] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt');
[samples_X_extrapol , samples_Y_extrapol , nr_samples_extrapol] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_heavy_mass.txt');


% --------------------------------------------------------------------------------------------------------


% -------------- Change number of data for the training.

nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];

nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_error_extrapol = [];
var_train = [];
var_test = [];
var_extrapol = [];
var_mean_1 = [];
var_mean_2 = [];


for id_nb_data = nb_data_total

samples_X = [];
samples_Y = [];
operation = [ones(1,id_nb_data) zeros(1,nr_samples_total - id_nb_data) ];  %Fill the vector with 0 and 1
operation = operation(randperm(nr_samples_total ));  %Randomly reorder it
nb_column = 0;
for id = operation
   nb_column = nb_column + 1;
   if id == 1 
       samples_X = cat(2,samples_X, samples_X_total(:,nb_column));
       samples_Y = cat(2,samples_Y, samples_Y_total(:,nb_column));
   end    
end    



hiddim = 1500;
reg = 1e-5;



bipidx = 0;
fw = 1;
	
disp(['Config: BIP: ' num2str(bipidx)  ' fw: ' num2str(fw) ' reg: ' num2str(reg) ' hiddim: ' num2str(hiddim)])



% --------------------------------------------------------------------------------------------------------

% ------------- Cross validation -----------------------------------------

% --------------------------------------------------------------------------------------------------------




%elm:
%BIP - Batch intrinsic plasticity (extension for tuning of parameters)
%reg - regularization for linear output learning
%hidDim - hidden layer size (random layer)
%
elm_learner=ELM(7,6,{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});


% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_test = 10;
errors_test=zeros(nb_test,6);
errors_extrapol=zeros(nb_test,6);
var_errors_test=zeros(nb_test,6);
var_errors_extrapol=zeros(nb_test,6);
%{
for idx = 2:6
    cat(3,errors_test,zeros(nb_test,1));
    cat(3,errors_extrapol,zeros(nb_test,1));
    cat(3,var_errors_test,zeros(nb_test,1));
    cat(3,var_errors_extrapol,zeros(nb_test,1));
end
%}
for i=1:nb_test

nb_fold = 5;
c = cvpartition(id_nb_data,'KFold' , nb_fold);



nr_train_s = c.TrainSize;
nr_test_s = c.TestSize;

if i == 1
    nb_train_data = [nb_train_data nr_train_s(1)];
end

n=1;
    trIdx=find(training(c,n));
	teIdx=find(test(c,n));
	nr_train_samples = nr_train_s(n);
	nr_test_samples = nr_test_s(n);
	samples_train_X = [];
	samples_train_Y = [];


% Random data chosen to train
	for idx = trIdx
		samples_train_X = [samples_train_X samples_X(: , idx )];
		samples_train_Y = [samples_train_Y samples_Y(: , idx )];
	end	

% -----------------------------------------------------------------------------------------------------



% ------------- Training -----------------------------------------------------------------------------



	if (n == 1)
		elm_learner.a=fw.*elm_learner.a;
%scale inputs to range etc:
%elm_learner.b sind die centers der fermi funktionen - alle random
%elm_learner.a 'width' of fermi function, default: 1
%the larger 'a' gets, the 'smaller' the fermi function


%elm_learner.a=0.01.*elm_learner.a; % -> extremely wide fermi function
%almost linear

%elm_learner.a=5.*elm_learner.a; % -> smaller sigmoid in function space
	end

	elm_learner.init(samples_train_X');
%train the stuff
	elm_learner.train(samples_train_X', samples_train_Y');


% ---------------------------------------------------------------------------------------------------




% ------------- Evaluation - error calculation ---------------------------------------------------------


%execute on testdata


[elm_result] = (elm_learner.apply(samples_X_total'))';
result_extrapol = elm_learner.apply(samples_X_extrapol')';
temp_test = (sum((elm_result - samples_Y_total).^2,2))/(nr_samples_total);
temp_extrapol = (sum((result_extrapol - samples_Y_extrapol).^2,2))/(nr_samples_extrapol);
for jntidx = 1:6
	errors_test(i,jntidx) = temp_test(jntidx);
    var_errors_test(i,jntidx) = var(abs(elm_result(jntidx,:) - samples_Y_total(jntidx,:)).^2);
    errors_extrapol(i,jntidx) = temp_extrapol(jntidx);
    var_errors_extrapol(i,jntidx) = var(abs(result_extrapol(jntidx,:) - samples_Y_extrapol(jntidx,:)).^2);
end    

end % end of nb_test loop 
        
        


temp_jnt_err = [];
temp_jnt_var = [];
temp_jnt_err_extrapol = [];
temp_jnt_var_extrapol = [];
var_err = [];
var_mean_extrapol = [];
for jntidx = 1:6
    temp_jnt_err = [temp_jnt_err mean(errors_test(:,jntidx))];
    temp_jnt_var = [temp_jnt_var mean(var_errors_test(:,jntidx))];
    temp_jnt_err_extrapol = [temp_jnt_err_extrapol mean(errors_extrapol(:,jntidx))];
    temp_jnt_var_extrapol = [temp_jnt_var_extrapol mean(var_errors_extrapol(:,jntidx))];
    var_err = [var_err var(errors_test(:,jntidx))];
    var_mean_extrapol = [var_mean_extrapol var(errors_extrapol(:,jntidx))];
end    



    jnt_error_test = [jnt_error_test temp_jnt_err'];
    jnt_error_extrapol = [jnt_error_extrapol temp_jnt_err_extrapol'];
    var_test = [var_test temp_jnt_var'];
    var_extrapol = [var_extrapol temp_jnt_var_extrapol'];
    var_mean_1 = [var_mean_1 var_err'];
    var_mean_2 = [var_mean_2 var_mean_extrapol'];

end







% --------------------------------------------------------------------------------------------------





% ------------- PLOT COMPARISON---------------------------------------------------------

figure();

% Normal test 
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_testy=[jnt_error_test(jntidx , (1:end))+var_mean_1(jntidx,:) flip(jnt_error_test(jntidx , (1:end))-var_mean_1(jntidx,:))];
    hold on;
    
    
    big_lineadata_var_x=[big_nb_train_data flip(big_nb_train_data)];
    big_lineadata_var_extrapol1y=[big_jnt_error_extrapol_1(jntidx , (1:end))+big_var_extrapol_1(jntidx,:) flip(big_jnt_error_extrapol_1(jntidx , (1:end))-big_var_extrapol_1(jntidx,:))];

    big_p2 = fill(big_lineadata_var_x,big_lineadata_var_extrapol1y,[0.80 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    
    
    p1 = fill(lineadata_var_x,lineadata_var_testy,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]

    g1 = plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)),'k');
    
    big_g2 = plot(big_nb_train_data(1:end) , big_jnt_error_extrapol_1(jntidx , (1:end)),'r');
    
    str_title=sprintf('Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g1, p1, big_g2 , big_p2], {'Simple Mean Test', 'Simple Variance test', 'Big Mean Test', 'Big Variance test'}, 'Location','southeast');
    set(gca,'Layer','top');
    xlim([nb_train_data(1),nb_train_data(end)]);  
end


figure();
% Normal extrapol
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    hold on;
    lineadata_var_extrapoly=[jnt_error_extrapol(jntidx , (1:end))+var_mean_2(jntidx,:) flip(jnt_error_extrapol(jntidx , (1:end))-var_mean_2(jntidx,:))];
    
    
    big_lineadata_var_x=[big_nb_train_data flip(big_nb_train_data)];
    big_lineadata_var_extrapol1y=[big_jnt_error_extrapol_1(jntidx , (1:end))+big_var_extrapol_1(jntidx,:) flip(big_jnt_error_extrapol_1(jntidx , (1:end))-big_var_extrapol_1(jntidx,:))];
    big_lineadata_var_extrapol2y=[big_jnt_error_extrapol_2(jntidx , (1:end))+big_var_extrapol_2(jntidx,:) flip(big_jnt_error_extrapol_2(jntidx , (1:end))-big_var_extrapol_2(jntidx,:))];
    big_p3 = fill(big_lineadata_var_x,big_lineadata_var_extrapol2y,[1 0.8 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    p3 = fill(lineadata_var_x,lineadata_var_extrapoly,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]

    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx , (1:end)));
    
    big_g3 = plot(big_nb_train_data(1:end) , big_jnt_error_extrapol_2(jntidx , (1:end)));
    
    str_title=sprintf('Extrapolation Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' );
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g3, p3, big_g3, big_p3 ], {'Simple Mean Extrapol' , 'Simple Variance extrapol' , 'Big Mean Extrapol' , 'Big Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top');
    xlim([nb_train_data(1),nb_train_data(end)]);  
end


figure();
% Test - stability
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_testy=[jnt_error_test(jntidx , (1:end))+var_test(jntidx,:) flip(jnt_error_test(jntidx , (1:end))-var_test(jntidx,:))];
    hold on;
    
    
    big_lineadata_var_x=[big_nb_train_data flip(big_nb_train_data)];
    big_lineadata_var_extrapol1y=[big_jnt_error_extrapol_1(jntidx , (1:end))+big_var_mean_extrapol_1(jntidx,:) flip(big_jnt_error_extrapol_1(jntidx , (1:end))-big_var_mean_extrapol_1(jntidx,:))];

    p1 = fill(lineadata_var_x,lineadata_var_testy,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    big_p2 = fill(big_lineadata_var_x,big_lineadata_var_extrapol1y,[0.80 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]

    g1 = plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)),'k');
    
    big_g2 = plot(big_nb_train_data(1:end) , big_jnt_error_extrapol_1(jntidx , (1:end)),'r');
    
    str_title=sprintf('Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g1, p1,big_g2 , big_p2], {'Simple Mean Test', 'Simple Variance test' , 'Big Mean Test', 'Big Variance test'}, 'Location','southeast');
    set(gca,'Layer','top');
    xlim([nb_train_data(1),nb_train_data(end)]);  
end


figure();
% extrapolation stability
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    hold on;
    lineadata_var_extrapoly=[jnt_error_extrapol(jntidx , (1:end))+var_extrapol(jntidx,:) flip(jnt_error_extrapol(jntidx , (1:end))-var_extrapol(jntidx,:))];
    
    
    big_lineadata_var_x=[big_nb_train_data flip(big_nb_train_data)];
    big_lineadata_var_extrapol2y=[big_jnt_error_extrapol_2(jntidx , (1:end))+big_var_mean_extrapol_2(jntidx,:) flip(big_jnt_error_extrapol_2(jntidx , (1:end))-big_var_mean_extrapol_2(jntidx,:))];
    p3 = fill(lineadata_var_x,lineadata_var_extrapoly,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    big_p3 = fill(big_lineadata_var_x,big_lineadata_var_extrapol2y,[1 0.8 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]

    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx , (1:end)));
    
    big_g3 = plot(big_nb_train_data(1:end) , big_jnt_error_extrapol_2(jntidx , (1:end)));
    
    str_title=sprintf('Extrapolating Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([ g3 , p3 , big_g3 , big_p3 ], {'Simple Mean Extrapol' , 'Simple Variance extrapol', 'Big Mean Extrapol' , 'Big Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top');
    xlim([nb_train_data(1),nb_train_data(end)]);  
end