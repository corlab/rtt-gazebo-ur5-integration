clear all;
close all;

%restoredefaultpath;
addpath('../learner/algorithms');
addpath('../learner/metric');
addpath('../learner/utils');



%{

%% Optimization of the parameters

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

%nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];
nb_data_total = [15625]; % To first find the right parameters for the ELM (regression and hidden dimension).

nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_error_extrapol = [];
var_train = [];
var_test = [];
var_extrapol = [];


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

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------

% ------------- Parameters optimization -----------------------------------------

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------


regularization=[1e-13 1e-11 1e-9 1e-7 1e-5 1e-3 1e-1 1];
hiddimcombinations=[20 50 75 100 250 500 750 1000 1250 1500 1750 2000 2250 2500 3000];

%regularization=[1e-10 1e-1];
%hiddimcombinations=[50 100];
BIP=[0 1];
fermiwidth=[1 20];

resultmatrix_test=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_test=zeros(size(regularization,2),size(hiddimcombinations,2));

resultmatrix_train=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_train=zeros(size(regularization,2),size(hiddimcombinations,2));

for idx = 2:6
    cat(3, resultmatrix_test , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_test , zeros(size(regularization,2),size(hiddimcombinations,2)));

    cat(3, resultmatrix_train , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_train , zeros(size(regularization,2),size(hiddimcombinations,2)));
end


idx_reg = 0;
idx_hiddim = 0;



nb_config = 0;
hiddim = 1500;
reg = 1e-5;


%massidx = 4;

bipidx = 0;
%for bipidx = BIP
 %  for fw = fermiwidth
   fw = 1;
		nb_config = nb_config + 1;
		idx_reg = 0;
		for reg=regularization
			idx_reg = idx_reg + 1;
			idx_hiddim = 0;
			for hiddim=hiddimcombinations
                                    
                
				idx_hiddim = idx_hiddim+1;
		


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
nb_fold = 5;
c = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test=zeros(nb_fold,1);
errors_train=zeros(nb_fold,1);
var_errors_test=zeros(nb_fold,1);
var_errors_train=zeros(nb_fold,1);
for idx = 2:6
    cat(3,errors_test,zeros(nb_fold,1));
    cat(3,errors_train,zeros(nb_fold,1));
    cat(3,var_errors_test,zeros(nb_fold,1));
    cat(3,var_errors_train,zeros(nb_fold,1));
end

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
	samples_test_X = [];
	samples_test_Y = [];


% Random data chosen to train
	for idx = trIdx
		samples_train_X = [samples_train_X samples_X(: , idx )];
		samples_train_Y = [samples_train_Y samples_Y(: , idx )];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X = [samples_test_X samples_X(: , idx )];
		samples_test_Y = [samples_test_Y samples_Y(: , idx )];
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

	[elm_result] = (elm_learner.apply(samples_test_X'))';
	[elm_training_result] = (elm_learner.apply(samples_train_X'))';


    
    
%compute errors for each joint

temp_test = (sum((elm_result - samples_test_Y).^2,2))/(nr_test_samples);
temp_train = (sum((elm_training_result - samples_train_Y).^2,2))/(nr_train_samples);
for jntidx = 1:6
	errors_test(n,jntidx) = temp_test(jntidx);
	errors_train(n,jntidx) = temp_train(jntidx);
    var_errors_test(n,jntidx) = var(abs(elm_result(jntidx,:) - samples_test_Y(jntidx,:)));
	var_errors_train(n,jntidx) = var(abs(elm_training_result(jntidx,:) - samples_train_Y(jntidx,:)));
end    

end
        
        
    temp_train_error = [];
    temp_test_error = [];
    var_extrapol_jnt = [];
    result_extrapol = elm_learner.apply(samples_X_extrapol')';
        for jntidx = 1:6
            resultmatrix_test(idx_reg,idx_hiddim , jntidx)=mean(errors_test(:,jntidx));
            resultmatrix_var_test(idx_reg,idx_hiddim , jntidx)=mean(var_errors_test(:,jntidx));
            resultmatrix_train(idx_reg,idx_hiddim, jntidx)=mean(errors_train(:,jntidx));
            resultmatrix_var_train(idx_reg,idx_hiddim , jntidx)=mean(var_errors_train(:,jntidx));
            temp_train_error = [temp_train_error ; mean(errors_train(:,jntidx))];
            temp_test_error = [temp_test_error ; mean(errors_test(:,jntidx))];
            var_extrapol_jnt = [var_extrapol_jnt ; var(abs(result_extrapol(jntidx,:) - samples_Y_extrapol(jntidx,:)))];
        end    

    extrapol_err =  (sum(((elm_learner.apply(samples_X_extrapol'))' - samples_Y_extrapol).^2,2))/(nr_samples_extrapol);   
    jnt_error_train = [jnt_error_train temp_train_error];
    jnt_error_test = [jnt_error_test temp_test_error];
    jnt_error_extrapol = [jnt_error_extrapol extrapol_err];
    var_train = [var_train reshape(resultmatrix_var_test(idx_reg,idx_hiddim , :),[6 1])];
    var_test = [var_test reshape(resultmatrix_var_train(idx_reg,idx_hiddim , :),[6 1])];
    var_extrapol = [var_extrapol var_extrapol_jnt];
end
end


    
	
    str_title=sprintf('elm error - BIP = %d and fermiwidth = %d', bipidx, fw);

	figure_saver(nb_config) = figure('Name',str_title,'NumberTitle','on');
	rotate3d on


    for jntidx = 1:6
 
	subplot(4,6,jntidx);
	surf(resultmatrix_test(:,:,jntidx))
	title('Mean Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

	subplot(4,6,jntidx+6);
	surf(resultmatrix_var_test(:,:,jntidx))
	title('Variance Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

	subplot(4,6,jntidx+12);
	surf(resultmatrix_train(:,:,jntidx))
	title('Mean Error train')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

	subplot(4,6,jntidx+18);
	surf(resultmatrix_var_train(:,:,jntidx))
	title('Variance Error train')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

    end
   
    
    
    resultmatrix_test=zeros(size(regularization,2), size(hiddimcombinations,2));
    resultmatrix_var_test=zeros(size(regularization,2),size(hiddimcombinations,2));

    resultmatrix_train=zeros(size(regularization,2), size(hiddimcombinations,2));
    resultmatrix_var_train=zeros(size(regularization,2),size(hiddimcombinations,2));
for idx = 2:6
    cat(3, resultmatrix_test , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_test , zeros(size(regularization,2),size(hiddimcombinations,2)));

    cat(3, resultmatrix_train , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_train , zeros(size(regularization,2),size(hiddimcombinations,2)));
end

%end
%end


savefig(figure_saver , 'elm_results.fig' , 'compact');
close(figure_saver);

 

end


%-------------------------------------------------------------------------------------------

% Plot torque as function of angle. massidx must be specified.

%plotData(samples_X , samples_Y , massidx , samples_test_X , samples_test_Y , elm_learner);
%-------------------------------------------------------------------------------------------



% --------------------------------------------------------------------------------------------------



% ------------- Writing parameters in file ---------------------------------------------------------

%cppModelWrite(elm_learner , massidx);

%plot(nb_data_total , jnt_error_train);


for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_testy=[jnt_error_test(jntidx , (1:end))+var_test(jntidx,:) flip(jnt_error_test(jntidx , (1:end))-var_test(jntidx,:))];
    lineadata_var_trainy=[jnt_error_train(jntidx , (1:end))+var_train(jntidx,:) flip(jnt_error_train(jntidx , (1:end))-var_train(jntidx,:))];
    p2 = fill(lineadata_var_x,lineadata_var_trainy,[0.80 1 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    lineadata_var_extrapoly=[jnt_error_extrapol(jntidx , (1:end))+var_extrapol(jntidx,:) flip(jnt_error_extrapol(jntidx , (1:end))-var_extrapol(jntidx,:))];
    hold on;
    p3 = fill(lineadata_var_x,lineadata_var_extrapoly,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    p1 = fill(lineadata_var_x,lineadata_var_testy,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    g1 = plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)),'k');
    hold on;
    g2 = plot(nb_train_data(1:end) , jnt_error_train(jntidx , (1:end)));
    hold on;
    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx , (1:end)));
    
    
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g1, p1 , g2 , p2 , g3 , p3], {'Mean Test', 'Variance test' , 'Mean Train' , 'Variance train' , 'Mean Extrapol' , 'Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])

    
    
end

%}



%% Testing plot

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
var_mean_3 = [];


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
errors_train = zeros(nb_test,6);
var_errors_train = zeros(nb_test,6);

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
result_train = elm_learner.apply(samples_train_X')';
temp_test = (sum((elm_result - samples_Y_total).^2,2))/(nr_samples_total);
temp_extrapol = (sum((result_extrapol - samples_Y_extrapol).^2,2))/(nr_samples_extrapol);
temp_train = (sum((result_train - samples_train_Y).^2,2))/(size(samples_train_Y , 2));
for jntidx = 1:6
	errors_test(i,jntidx) = temp_test(jntidx);
    var_errors_test(i,jntidx) = var(abs(elm_result(jntidx,:) - samples_Y_total(jntidx,:)).^2);
    errors_extrapol(i,jntidx) = temp_extrapol(jntidx);
    var_errors_extrapol(i,jntidx) = var(abs(result_extrapol(jntidx,:) - samples_Y_extrapol(jntidx,:)).^2);
    errors_train(i,jntidx) = temp_train(jntidx);
    var_errors_train(i,jntidx) = var(abs(result_train(jntidx,:) - samples_train_Y(jntidx,:)).^2);
end    

end % end of nb_test loop 
        
        


temp_jnt_err = [];
temp_jnt_var = [];
temp_jnt_err_extrapol = [];
temp_jnt_var_extrapol = [];
temp_jnt_err_train = [];
temp_jnt_var_train = [];
var_err = [];
var_mean_extrapol = [];
var_err_train = [];
for jntidx = 1:6
    temp_jnt_err = [temp_jnt_err mean(errors_test(:,jntidx))];
    temp_jnt_var = [temp_jnt_var mean(var_errors_test(:,jntidx))];
    temp_jnt_err_extrapol = [temp_jnt_err_extrapol mean(errors_extrapol(:,jntidx))];
    temp_jnt_var_extrapol = [temp_jnt_var_extrapol mean(var_errors_extrapol(:,jntidx))];
    temp_jnt_err_train = [temp_jnt_err_train mean(errors_train(:,jntidx))];
    temp_jnt_var_train = [temp_jnt_var_train mean(var_errors_train(:,jntidx))];
    var_err = [var_err var(errors_test(:,jntidx))];
    var_mean_extrapol = [var_mean_extrapol var(errors_extrapol(:,jntidx))];
    var_err_train = [var_err_train var(errors_train(:,jntidx))];
end    



    jnt_error_test = [jnt_error_test temp_jnt_err'];
    jnt_error_extrapol = [jnt_error_extrapol temp_jnt_err_extrapol'];
    jnt_error_train = [jnt_error_train temp_jnt_err_train'];
    var_test = [var_test temp_jnt_var'];
    var_extrapol = [var_extrapol temp_jnt_var_extrapol'];
    var_train = [var_train temp_jnt_var_train'];
    var_mean_1 = [var_mean_1 var_err'];
    var_mean_2 = [var_mean_2 var_mean_extrapol'];
    var_mean_3 = [var_mean_3 var_err_train'];

end



figure();
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_testy=[jnt_error_test(jntidx , (1:end))+var_test(jntidx,:) flip(jnt_error_test(jntidx , (1:end))-var_test(jntidx,:))];
    lineadata_var_trainy=[jnt_error_train(jntidx , (1:end))+var_train(jntidx,:) flip(jnt_error_train(jntidx , (1:end))-var_train(jntidx,:))];
    hold on;
    p1 = fill(lineadata_var_x,lineadata_var_testy,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    p2 = fill(lineadata_var_x,lineadata_var_trainy,[0.80 1 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    g1 = plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)),'k');
    hold on;
    g2 = plot(nb_train_data(1:end) , jnt_error_train(jntidx , (1:end)));
    
    
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g1, p1 , g2 , p2 ], {'Mean Test', 'Variance test' , 'Mean Train' , 'Variance train'}, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])   
end


figure();
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_testy=[jnt_error_test(jntidx , (1:end))+var_mean_1(jntidx,:) flip(jnt_error_test(jntidx , (1:end))-var_mean_1(jntidx,:))];
    lineadata_var_trainy=[jnt_error_train(jntidx , (1:end))+var_mean_3(jntidx,:) flip(jnt_error_train(jntidx , (1:end))-var_mean_3(jntidx,:))];
    p2 = fill(lineadata_var_x,lineadata_var_trainy,[0.80 1 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    p1 = fill(lineadata_var_x,lineadata_var_testy,[0.90 0.90 0.90], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    g1 = plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)),'k');
    hold on;
    g2 = plot(nb_train_data(1:end) , jnt_error_train(jntidx , (1:end)));
    
    
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g1, p1 , g2 , p2 ], {'Mean Test', 'Variance test' , 'Mean Train' , 'Variance train' }, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])   
end


figure();
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
   
    lineadata_var_extrapoly=[jnt_error_extrapol(jntidx , (1:end))+var_extrapol(jntidx,:) flip(jnt_error_extrapol(jntidx , (1:end))-var_extrapol(jntidx,:))];
    hold on;
    p3 = fill(lineadata_var_x,lineadata_var_extrapoly,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx , (1:end)));
    
    
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g3 , p3], {'Mean Extrapol' , 'Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])   
end


figure();
for jntidx = 1:6
    subplot(2,3,jntidx);
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
   
    lineadata_var_extrapoly=[jnt_error_extrapol(jntidx , (1:end))+var_mean_2(jntidx,:) flip(jnt_error_extrapol(jntidx , (1:end))-var_mean_2(jntidx,:))];
    hold on;
    p3 = fill(lineadata_var_x,lineadata_var_extrapoly,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    hold on;
    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol(jntidx , (1:end)));
    
    
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train' , 'Mean error extrapol');
    xlabel('Number of inputs');
    title(str_title);
    
    
    hold on;
   
    legend_handle = legend([g3 , p3], { 'Mean Extrapol' , 'Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])   
end