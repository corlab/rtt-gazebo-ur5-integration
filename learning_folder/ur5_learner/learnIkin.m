clear all;
close all;

%restoredefaultpath;
addpath('../learner/algorithms');
addpath('../learner/metric');
addpath('../learner/utils');

nr_samples = [];

range=[-2*pi,2*pi];


% -------------  Preparing data set --------------------------------------------------

%fileID = fopen('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/test_data_no_mass_set1.txt','r');

% Data set : 
samples_train_X = [];
samples_train_Y = [];

samples_test_X = [];   
samples_test_Y = [];

samples_X_total = [];
samples_Y_total = [];



[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass0.txt');
%[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/test_data.txt');

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
% --------------------------------------------------------------------------------------------------------


% -------------- Change number of data for the training.

nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];

nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
samples_X = [];
samples_Y = [];

for id_nb_data = nb_data_total


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

% ------------- Parameters optimization -----------------------------------------

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------


regularization=[1e-11 1e-10 1e-9 1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 1e-1 1 10];
hiddimcombinations=[20 50 75 100 250 500 750 1000 1750 2500];

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
hiddim = 500;
reg = 1e-5;


massidx = 4;

bipidx = 0;
%for bipidx = BIP
 %  for fw = fermiwidth
   fw = 1;
		nb_config = nb_config + 1;
		idx_reg = 0;
	%	for reg=regularization
			idx_reg = idx_reg + 1;
			idx_hiddim = 0;
	%		for hiddim=hiddimcombinations
                
               % if (bip ~= 1 && hiddim == 8000)
                    
                
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
elm_learner=ELM(6,6,{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});






% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_fold = 5;
%c = cvpartition(nr_samples(1),'KFold' , nb_fold);
c = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test=zeros(nb_fold,1);
errors_train=zeros(nb_fold,1);
for idx = 2:6
    cat(3,errors_test,zeros(nb_fold,1));
    cat(3,errors_train,zeros(nb_fold,1));
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
		samples_train_X = [samples_train_X samples_X(: , idx , massidx)];
		samples_train_Y = [samples_train_Y samples_Y(: , idx , massidx)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X = [samples_test_X samples_X(: , idx , massidx)];
		samples_test_Y = [samples_test_Y samples_Y(: , idx , massidx)];
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
% Unused now
%{ 
	test_error = sum( abs(elm_result - samples_test_Y) , 2)/nr_test_samples

	[elm_training_result] = (elm_learner.apply(samples_train_X'))';

	training_error = sum( abs(elm_training_result - samples_train_Y) , 2)/nr_train_samples
%}
%compute error with euclidian distance

for jntidx = 1:6
    temp_test = sqrt(sum((elm_result - samples_test_Y).^2,2))/(nr_test_samples);
    temp_train = sqrt(sum((elm_training_result - samples_train_Y).^2,2))/(nr_train_samples);
	errors_test(n,jntidx) = temp_test(jntidx);
	errors_train(n,jntidx) = temp_train(jntidx);
end    

end

    temp_train_error = [];
    temp_test_error = [];
        for jntidx = 1:6
            resultmatrix_test(idx_reg,idx_hiddim , jntidx)=mean(errors_test(:,jntidx));
            resultmatrix_var_test(idx_reg,idx_hiddim , jntidx)=var(errors_test(:,jntidx));
            resultmatrix_train(idx_reg,idx_hiddim, jntidx)=mean(errors_train(:,jntidx));
            resultmatrix_var_train(idx_reg,idx_hiddim , jntidx)=var(errors_train(:,jntidx));
            temp_train_error = [temp_train_error ; mean(errors_train(:,jntidx))];
            temp_test_error = [temp_test_error ; mean(errors_test(:,jntidx))];
        end    

        
    jnt_error_train = [jnt_error_train temp_train_error];
    jnt_error_test = [jnt_error_test temp_test_error];
    
%end
%end


    %{
	str_title=sprintf('elm error - BIP = %d and fermiwidth = %d', bipidx, fw);
	%suptitle(str_title);

	%figure(nb_config)
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
%}


end


%-------------------------------------------------------------------------------------------

% Plot torque as function of angle. massidx must be specified.

%plotData(samples_X , samples_Y , massidx , samples_test_X , samples_test_Y , elm_learner);
%-------------------------------------------------------------------------------------------



% --------------------------------------------------------------------------------------------------



% ------------- Writing parameters in file ---------------------------------------------------------

cppModelWrite(elm_learner , massidx);

%plot(nb_data_total , jnt_error_train);
for jntidx = 1:6
    subplot(2,3,jntidx);
    plot(nb_train_data(3:end) , jnt_error_test(jntidx , (3:end)));
    hold on;
    plot(nb_train_data(3:end) , jnt_error_train(jntidx , (3:end)));
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train');
    xlabel('Number of inputs');
    title(str_title);
end