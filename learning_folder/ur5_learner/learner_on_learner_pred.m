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



[samples_pos_temp  , samples_X_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass0.txt');
samples_X_total = cat(3, samples_X_total ,samples_X_temp); % Torque
samples_Y_total = cat(3 , samples_Y_total, 0.0001*ones(size(samples_X_temp,1)),1); % Mass
samples_pos_total = cat(3,samples_pos_total , samples_pos_temp); % Position
nr_samples = [nr_samples nr_samples_temp];



[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass1.txt');
samples_X_total = cat(3, samples_X_total ,samples_X_temp); % Torque
samples_Y_total = cat(3 , samples_Y_total, 1*ones(size(samples_X_temp,1)),1); % Mass
samples_pos_total = cat(3,samples_pos_total , samples_pos_temp); % Position
nr_samples = [nr_samples nr_samples_temp];

[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass5.txt');
samples_X_total = cat(3, samples_X_total ,samples_X_temp); % Torque
samples_Y_total = cat(3 , samples_Y_total, 5*ones(size(samples_X_temp,1)),1); % Mass
samples_pos_total = cat(3,samples_pos_total , samples_pos_temp); % Position
nr_samples = [nr_samples nr_samples_temp];

[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_mass3.txt');
samples_X_total = cat(3, samples_X_total ,samples_X_temp); % Torque
samples_Y_total = cat(3 , samples_Y_total, 3*ones(size(samples_X_temp,1)),1); % Mass
samples_pos_total = cat(3,samples_pos_total , samples_pos_temp); % Position
nr_samples = [nr_samples nr_samples_temp];


% --------------------------------------------------------------------------------------------------------


% -------------- Change number of data for the training.

%nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];
nb_data_total = [15625];


nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_eror_extrapol_1 = [];
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


regularization=[1e-11 1e-10 1e-9 1e-8 1e-7 1e-3 1e-1 1 10];
hiddimcombinations=[1 3 5 7 10 15 20 50 100 250 500 750 1000 1250 1500 1750 2000];

%regularization=[1e-10 1e-1];
%hiddimcombinations=[50 100];
BIP=[0 1];
fermiwidth=[1 20];

resultmatrix_test=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_test=zeros(size(regularization,2),size(hiddimcombinations,2));

resultmatrix_train=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_train=zeros(size(regularization,2),size(hiddimcombinations,2));



idx_reg = 0;
idx_hiddim = 0;



nb_config = 0;
hiddim = 500;
reg = 1e-3;


%massidx = 4;

bipidx = 1;
%for bipidx = BIP
 %  for fw = fermiwidth
   fw = 20;
		nb_config = nb_config + 1;
		idx_reg = 0;
		for reg=regularization
			idx_reg = idx_reg + 1;
			idx_hiddim = 0;
			for hiddim=hiddimcombinations
                
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
elm_learner=ELM(6,1,{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});






% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_fold = 5;
%c = cvpartition(nr_samples(1),'KFold' , nb_fold);
c = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test=zeros(nb_fold,1);
errors_train=zeros(nb_fold,1);

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


    % Data prepared for each payload
    
% Random data chosen to train
	for idx = trIdx
		samples_train_X_0 = [samples_train_X samples_X(: , idx , 1)];
		samples_train_Y_0 = [samples_train_Y samples_Y(: , idx , 1)];
	end	

% Random data chosen to test
	for idx = teIdx
		samples_test_X_0 = [samples_test_X samples_X(: , idx , 1)];
		samples_test_Y_0 = [samples_test_Y samples_Y(: , idx , 1)];
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
	elm_array = [];
    
   for i=1:size(samples_pos_total,1)
   		elm_array(i) = elm_learner.copy();
   		elm_array(i).train(samples_train_X(i,:,:)' , samples_train_Y(i,:,:)'); 



    
% ELM for ELM    
    elm_on_elm=ELM(6, size(reshape(elm_learner.wOut , [size(elm_learner.wOut , 1)*size(elm_learner.wOut , 2),1]) , 1),{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});
    big_elm_X = samples_pos_total(:,:,1);
    big_elm_Y = [];
    for i=1:size(samples_pos_total,1)
    	big_elm_Y = [big_elm_Y , reshape(elm_array(i).wOut,[size(elm_array(i).wOut,1)*size(elm_array(i).wOut,1)])];
    elm_on_elm.init(big_elm_X' , big_elm_Y');
    elm_on_elm.train(big_elm_X' , big_elm_Y');
    
% ---------------------------------------------------------------------------------------------------




% ------------- Evaluation - error calculation ---------------------------------------------------------

%% Evaluation for learned data.






%% Evaluation for different payloads


% Payloads from 0 to 5 kg and payloads from 5 to 10 kg.
[extra_test_X_1 , extra_test_Y_1 , nr_samples_1] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt'); %% A completer avec des donnees pour des masses de 5 a 10.
%[extra_test_X_2 , extra_test_Y_2 , nr_samples_2] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt'); %% A completer avec des donnees pour des masses de 5 a 10.


%%%%%%%%%%%% Error computation on extrapolated data (payload from 5kg to 10kg)
%%%%%%%%%% Done once parameters are found regularization and hidden dimension).

extra_data_error = [];

for idx = 1:size(extra_test_X_1 , 2)
    elm_learner.wOut = reshape(elm_on_elm.apply(extra_test_X_1(1:6 , idx )) , [size(elm_learner.wOut)]);
    elm_result = elm_learner.apply(extra_test_Y_1(1:6 , idx )');
    extra_data_error = [extra_data_error (elm_result - extra_test_X_1(7,idx)')']; 
end
%extra_data_error = sqrt(extra_data_error)/size(extra_test_X_1 , 2);
        
    resultmatrix_test(idx_reg,idx_hiddim)=mean(extra_data_error);
    resultmatrix_var_test(idx_reg,idx_hiddim=var(extra_data_error);

	jnt_error_extrapol_1 = [jnt_error_extrapol_1 extra_data_error];    


end




end
end



   
    
    


%end
%end




end;




 
	str_title=sprintf('elm error - BIP = %d and fermiwidth = %d', bipidx, fw);

	figure_saver(nb_config) = figure('Name',str_title,'NumberTitle','on');
	rotate3d on
 
	subplot(1,2,1);
	surf(resultmatrix_test(:,:))
	title('Mean Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

	subplot(1,2,2);
	surf(resultmatrix_var_test(:,:))
	title('Variance Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;


    end
      
    savefig(figure_saver , 'elm_results.fig' , 'compact');
	close(figure_saver);    


%-------------------------------------------------------------------------------------------

% Plot torque as function of angle. massidx must be specified.

%plotData(samples_X , samples_Y , massidx , samples_test_X , samples_test_Y , elm_learner);
%-------------------------------------------------------------------------------------------



% --------------------------------------------------------------------------------------------------



% ------------- Writing parameters in file ---------------------------------------------------------

%cppModelWrite(elm_learner , massidx);

%plot(nb_data_total , jnt_error_train);
    plot(nb_train_data(1:end) , jnt_error_test( (1:end)));
    hold on;
    plot(nb_train_data(1:end) , jnt_error_train((1:end)));
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs');
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train');
    xlabel('Number of inputs * 4(nb of payloads)');
    title(str_title);
end
