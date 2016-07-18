clear all;
close all;

%restoredefaultpath;
addpath('../learner/algorithms');
addpath('../learner/metric');
addpath('../learner/utils');


%{
%% Optimization
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


% Evaluation for different payloads


% Payloads from 0 to 5 kg and payloads from 5 to 10 kg.
[extra_test_X_1 , extra_test_Y_1 , nr_samples_1] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_all_mass.txt'); 
[extra_test_X_2 , extra_test_Y_2 , nr_samples_2] = readData_mass('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/random_positions/good_data/test_data_heavy_mass.txt');


% --------------------------------------------------------------------------------------------------------


% -------------- Change number of data for the training.

%nb_data_total = [10 50 100 500 1000 2500 5000 7500 8500 10000 11000 12500 15000 15625];
nb_data_total = [15625];
%nb_data_total = [10 500 1500 3000 10000 15625];

nb_train_data = [];
jnt_error_train = [];
jnt_error_test = [];
jnt_error_extrapol_1 = [];
jnt_error_extrapol_2 = [];
var_train = [];
var_test = [];
var_extrapol_1 = [];
var_extrapol_2 = [];



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


%regularization=[1e-11 1e-10 1e-9 1e-8 1e-7 1e-3 1e-1 1 10];
%hiddimcombinations=[1 3 5 7 10 15 20 50 100 250 500 750 1000 1250 1500 1750 2000];

regularization=[1e-11 1e-9 1e-7 1e-5 1e-3 1e-1 1 10];
hiddimcombinations=[1 5 10 15 20 50 100 250 500 750 1000 1500 2000];

%regularization=[1e-11 1e-10 1e-9 1e-8 1e-7];
%hiddimcombinations=[10 20 30 40 50 60 70 80 90 100];

%regularization=[1e-11 1e-1];
%hiddimcombinations=[5 500];

%regularization=[1e-10 1e-1];
%hiddimcombinations=[50 100];
BIP=[0 1];
fermiwidth=[1 20];


% For random data testing
resultmatrix_test=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_test=zeros(size(regularization,2),size(hiddimcombinations,2));
resultmatrix_train=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_train=zeros(size(regularization,2),size(hiddimcombinations,2));

resultmatrix_test_2=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_test_2=zeros(size(regularization,2),size(hiddimcombinations,2));
resultmatrix_train_2=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_train_2=zeros(size(regularization,2),size(hiddimcombinations,2));

for idx = 2:6
    cat(3, resultmatrix_test , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_test , zeros(size(regularization,2),size(hiddimcombinations,2)));
    cat(3, resultmatrix_train , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_train , zeros(size(regularization,2),size(hiddimcombinations,2)));
    
    cat(3, resultmatrix_test_2 , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_test_2 , zeros(size(regularization,2),size(hiddimcombinations,2)));
    cat(3, resultmatrix_train_2 , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_train_2 , zeros(size(regularization,2),size(hiddimcombinations,2)));
end


% For training and non random data testing

resultmatrix_test_0=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_test_0=zeros(size(regularization,2),size(hiddimcombinations,2));
resultmatrix_train_0=zeros(size(regularization,2), size(hiddimcombinations,2));
resultmatrix_var_train_0=zeros(size(regularization,2),size(hiddimcombinations,2));

for idx = 2:6
    cat(3, resultmatrix_test_0 , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_test_0 , zeros(size(regularization,2),size(hiddimcombinations,2)));
    cat(3, resultmatrix_train_0 , zeros(size(regularization,2), size(hiddimcombinations,2)));
    cat(3, resultmatrix_var_train_0 , zeros(size(regularization,2),size(hiddimcombinations,2)));
end





idx_reg = 0;
idx_hiddim = 0;



nb_config = 0;
hiddim = 60;
reg = 1e-8;


%massidx = 4;

bipidx = 1;
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
elm_learner=ELM(6,6,{'BIP',0;'reg', 1e-3; 'hidDim', 500});






% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_fold = 5;
c = cvpartition(id_nb_data,'KFold' , nb_fold);

errors_test_0=zeros(nb_fold,1);
errors_train_0=zeros(nb_fold,1);
var_errors_test_0=zeros(nb_fold,1);
var_errors_train_0=zeros(nb_fold,1);
errors_test_1=zeros(nb_fold,1);
errors_train_1=zeros(nb_fold,1);
var_errors_test_1=zeros(nb_fold,1);
var_errors_train_1=zeros(nb_fold,1);
errors_test_3=zeros(nb_fold,1);
errors_train_3=zeros(nb_fold,1);
var_errors_test_3=zeros(nb_fold,1);
var_errors_train_3=zeros(nb_fold,1);
errors_test_5=zeros(nb_fold,1);
errors_train_5=zeros(nb_fold,1);
var_errors_test_5=zeros(nb_fold,1);
var_errors_train_5=zeros(nb_fold,1);
for idx = 2:6
    cat(3,errors_test_0,zeros(nb_fold,1));
    cat(3,errors_train_0,zeros(nb_fold,1));
    cat(3,var_errors_test_0,zeros(nb_fold,1));
    cat(3,var_errors_train_0,zeros(nb_fold,1));
    cat(3,errors_test_1,zeros(nb_fold,1));
    cat(3,errors_train_1,zeros(nb_fold,1));
    cat(3,var_errors_test_1,zeros(nb_fold,1));
    cat(3,var_errors_train_1,zeros(nb_fold,1));
    cat(3,errors_test_3,zeros(nb_fold,1));
    cat(3,errors_train_3,zeros(nb_fold,1));
    cat(3,var_errors_test_3,zeros(nb_fold,1));
    cat(3,var_errors_train_3,zeros(nb_fold,1));
    cat(3,errors_test_5,zeros(nb_fold,1));
    cat(3,errors_train_5,zeros(nb_fold,1));
    cat(3,var_errors_test_5,zeros(nb_fold,1));
    cat(3,var_errors_train_5,zeros(nb_fold,1));
end

nr_train_s = c.TrainSize;
nr_test_s = c.TestSize;

nb_train_data = [nb_train_data nr_train_s(1)];


for n=1:nb_fold
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

%% Evaluation for learned data.

%{
%For each payload

% Right wOut
elm_learner.wOut = reshape(elm_on_elm.apply(0.0001) , [size(elm_learner.wOut)]);



% Test
%execute on testdata
	[elm_result_0] = (elm_learner.apply(samples_test_X_0'))';
	[elm_training_result_0] = (elm_learner.apply(samples_train_X_0'))';

% Right wOut
elm_learner.wOut = reshape(elm_on_elm.apply(1) , [size(elm_learner.wOut)]);

% Test
%execute on testdata
	[elm_result_1] = (elm_learner.apply(samples_test_X_1'))';
	[elm_training_result_1] = (elm_learner.apply(samples_train_X_1'))';
    
% Right wOut
elm_learner.wOut = reshape(elm_on_elm.apply(3) , [size(elm_learner.wOut)]);

% Test
%execute on testdata
	[elm_result_3] = (elm_learner.apply(samples_test_X_3'))';
	[elm_training_result_3] = (elm_learner.apply(samples_train_X_3'))';
  
 % Right wOut
elm_learner.wOut = reshape(elm_on_elm.apply(5) , [size(elm_learner.wOut)]);

% Test
%execute on testdata
	[elm_result_5] = (elm_learner.apply(samples_test_X_5'))';
	[elm_training_result_5] = (elm_learner.apply(samples_train_X_5'))';   
    
    
    
%compute errors for each joint

%compute error with euclidian distance

% Payload 0
temp_test = (sum((elm_result_0 - samples_test_Y_0).^2,2))/(nr_test_samples);
temp_train = (sum((elm_training_result_0 - samples_train_Y_0).^2,2))/(nr_train_samples);
for jntidx = 1:6
	errors_test_0(n,jntidx) = temp_test(jntidx);
	errors_train_0(n,jntidx) = temp_train(jntidx);
    var_errors_test_0(n,jntidx) = var(abs(elm_result_0(jntidx,:) - samples_test_Y_0(jntidx,:)));
	var_errors_train_0(n,jntidx) = var(abs(elm_training_result_0(jntidx,:) - samples_train_Y_0(jntidx,:)));
end    
 % Payload 1    
temp_test = (sum((elm_result_1 - samples_test_Y_1).^2,2))/(nr_test_samples);
temp_train = (sum((elm_training_result_1 - samples_train_Y_1).^2,2))/(nr_train_samples);
for jntidx = 1:6
	errors_test_1(n,jntidx) = temp_test(jntidx);
	errors_train_1(n,jntidx) = temp_train(jntidx);
    var_errors_test_1(n,jntidx) = var(abs(elm_result_1(jntidx,:) - samples_test_Y_1(jntidx,:)));
	var_errors_train_1(n,jntidx) = var(abs(elm_training_result_1(jntidx,:) - samples_train_Y_1(jntidx,:)));
end  

        
   % Payload 3 
temp_test = (sum((elm_result_3 - samples_test_Y_3).^2,2))/(nr_test_samples);
temp_train = (sum((elm_training_result_3 - samples_train_Y_3).^2,2))/(nr_train_samples);
 for jntidx = 1:6
	errors_test_3(n,jntidx) = temp_test(jntidx);
	errors_train_3(n,jntidx) = temp_train(jntidx);
    var_errors_test_3(n,jntidx) = var(abs(elm_result_3(jntidx,:) - samples_test_Y_3(jntidx,:)));
	var_errors_train_3(n,jntidx) = var(abs(elm_training_result_3(jntidx,:) - samples_train_Y_3(jntidx,:)));
end   

   % Payload 5 
temp_test = (sum((elm_result_5 - samples_test_Y_5).^2,2))/(nr_test_samples);
temp_train = (sum((elm_training_result_5 - samples_train_Y_5).^2,2))/(nr_train_samples);    
for jntidx = 1:6
	errors_test_5(n,jntidx) = temp_test(jntidx);
	errors_train_5(n,jntidx) = temp_train(jntidx);
    var_errors_test_5(n,jntidx) = var(abs(elm_result_5(jntidx,:) - samples_test_Y_5(jntidx,:)));
	var_errors_train_5(n,jntidx) = var(abs(elm_training_result_5(jntidx,:) - samples_train_Y_5(jntidx,:)));
end 

end % WHY is this here ?? Verify if it is the end of nfold



        for jntidx = 1:6
            resultmatrix_test_0(idx_reg,idx_hiddim , jntidx)=mean(errors_test_0(:,jntidx));
            resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx)=mean(var_errors_test_0(:,jntidx));
            resultmatrix_train_0(idx_reg,idx_hiddim, jntidx)=mean(errors_train_0(:,jntidx));
            resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)=mean(var_errors_train_0(:,jntidx));

        end   
        
  
        for jntidx = 1:6
            resultmatrix_test_0(idx_reg,idx_hiddim , jntidx)= resultmatrix_test_0(idx_reg,idx_hiddim , jntidx) + mean(errors_test_1(:,jntidx));
            resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx)=resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx) + mean(var_errors_test_1(:,jntidx));
            resultmatrix_train_0(idx_reg,idx_hiddim, jntidx)=resultmatrix_train_0(idx_reg,idx_hiddim, jntidx) + mean(errors_train_1(:,jntidx));
            resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)=resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)+mean(var_errors_train_1(:,jntidx));
        end    
 

        for jntidx = 1:6
            resultmatrix_test_0(idx_reg,idx_hiddim , jntidx)= resultmatrix_test_0(idx_reg,idx_hiddim , jntidx) + mean(errors_test_3(:,jntidx));
            resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx)=resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx) + mean(var_errors_test_3(:,jntidx));
            resultmatrix_train_0(idx_reg,idx_hiddim, jntidx)=resultmatrix_train_0(idx_reg,idx_hiddim, jntidx) + mean(errors_train_3(:,jntidx));
            resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)=resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)+mean(var_errors_train_3(:,jntidx));
        end    
        
   


    temp_train_error = [];
    temp_test_error = [];
        for jntidx = 1:6
            resultmatrix_test_0(idx_reg,idx_hiddim , jntidx)= (resultmatrix_test_0(idx_reg,idx_hiddim , jntidx) + mean(errors_test_5(:,jntidx)))/4;
            resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx)=(resultmatrix_var_test_0(idx_reg,idx_hiddim , jntidx) + mean(var_errors_test_5(:,jntidx)))/4;
            resultmatrix_train_0(idx_reg,idx_hiddim, jntidx)=(resultmatrix_train_0(idx_reg,idx_hiddim, jntidx) + mean(errors_train_5(:,jntidx)))/4;
            resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)=(resultmatrix_var_train_0(idx_reg,idx_hiddim , jntidx)+mean(var_errors_train_5(:,jntidx)))/4;
        end            
        
        
    jnt_error_train = [jnt_error_train resultmatrix_train_0(idx_reg,idx_hiddim , :)];
    jnt_error_test = [jnt_error_test resultmatrix_test_0(idx_reg,idx_hiddim , :)];
    var_train = [var_train resultmatrix_var_train_0(idx_reg,idx_hiddim , :)];
    var_test = [var_test resultmatrix_var_test_0(idx_reg,idx_hiddim , :)];
    %}
end % end of nb fold

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
        resultmatrix_test(idx_reg,idx_hiddim,jntidx)=(sum((extra_data_error(jntidx,:)).^2,2))/nr_samples_1;
        resultmatrix_var_test(idx_reg,idx_hiddim , jntidx)=var(extra_data_error(jntidx,:).^2);
    end
    
    for idx = 1:size(extra_test_X_2 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(extra_test_X_2(7 , idx )) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(extra_test_X_2(1:6 , idx )');
        extra_data_error_2 = [extra_data_error_2 (elm_result - extra_test_Y_2(:,idx)')'];    
    end
    for jntidx = 1:6
        resultmatrix_test_2(idx_reg,idx_hiddim,jntidx)=(sum((extra_data_error_2(jntidx,:)).^2,2))/nr_samples_2;
        resultmatrix_var_test_2(idx_reg,idx_hiddim , jntidx)=var(extra_data_error_2(jntidx,:).^2);
    end

    jnt_error_extrapol_1 = [jnt_error_extrapol_1 reshape(resultmatrix_test(idx_reg,idx_hiddim,:), [6,1])];
    jnt_error_extrapol_2 = [jnt_error_extrapol_2 reshape(resultmatrix_test_2(idx_reg,idx_hiddim,:), [6,1])];
    var_extrapol_1 = [var_extrapol_1 reshape(resultmatrix_var_test(idx_reg,idx_hiddim , :), [6,1])];
    var_extrapol_2 = [var_extrapol_2 reshape(resultmatrix_var_test_2(idx_reg,idx_hiddim , :), [6,1])];

        end  % end of hiddim
        end  % end of regularization
            

        str_title=sprintf('elm error - BIP = %d and fermiwidth = %d', bipidx, fw);

	figure_saver(nb_config) = figure('Name',str_title,'NumberTitle','on');
	rotate3d on


    for jntidx = 1:6
 
	subplot(2,6,jntidx);
	surf(resultmatrix_test(:,3:end,jntidx))
	title('Mean Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;

	subplot(2,6,jntidx+6);
	surf(resultmatrix_var_test(:,3:end,jntidx))
	title('Variance Error test')
    xlabel('hiddim')
    ylabel('regularization')
	rotate3d;


    end

    



    %{
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
    %}
%end
%end


savefig(figure_saver , 'elm_results.fig' , 'compact');
close(figure_saver);
    
  



%{
for jntidx = 1:6
    subplot(2,3,jntidx);
    plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)));
    hold on;
    plot(nb_train_data(1:end) , jnt_error_train(jntidx , (1:end)));
    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
    legend('Mean Error Test' , 'Mean Error Train');
    xlabel('Number of inputs * 4(nb of payloads)');
    title(str_title);
end
%}



end




        


%-------------------------------------------------------------------------------------------

% Plot torque as function of angle. massidx must be specified.

%plotData(samples_X , samples_Y , massidx , samples_test_X , samples_test_Y , elm_learner);
%-------------------------------------------------------------------------------------------



% --------------------------------------------------------------------------------------------------



% ------------- Writing parameters in file ---------------------------------------------------------
%{
%cppModelWrite(elm_learner , massidx);

%plot(nb_data_total , jnt_error_train);
for jntidx = 1:6
    subplot(2,3,jntidx);
    %plot(nb_train_data(1:end) , jnt_error_test(jntidx , (1:end)));
    hold on;
    %plot(nb_train_data(1:end) , jnt_error_train(jntidx , (1:end)));
    
    lineadata_var_x=[nb_train_data flip(nb_train_data)];
    lineadata_var_extrapol1y=[jnt_error_extrapol_1(jntidx , (1:end))+var_extrapol_1(jntidx,:) flip(jnt_error_extrapol_1(jntidx , (1:end))-var_extrapol_1(jntidx,:))];
    lineadata_var_extrapol2y=[jnt_error_extrapol_2(jntidx , (1:end))+var_extrapol_2(jntidx,:) flip(jnt_error_extrapol_2(jntidx , (1:end))-var_extrapol_2(jntidx,:))];
    p3 = fill(lineadata_var_x,lineadata_var_extrapol2y,[1 1 0.8], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    p2 = fill(lineadata_var_x,lineadata_var_extrapol1y,[0.80 1 1], 'FaceAlpha', 1.0, 'edgeAlpha', 0.0);  %' , 'FaceColor', [0.82 0.82 0.82]
    g3 = plot(nb_train_data(1:end) , jnt_error_extrapol_2(jntidx , (1:end)));
    g2 = plot(nb_train_data(1:end) , jnt_error_extrapol_1(jntidx , (1:end)));

    str_title=sprintf('Training and Testing Mean Errors as a function of number of inputs for joint %d', jntidx);
    ylabel('Mean Error');
  %  legend('Mean Error Test' , 'Mean Error Train' , 'Mean error on random payloads' , 'Mean error on high payloads');
    legend('Mean error on random payloads' , 'Mean error on high payloads');
    xlabel('Number of inputs * 4(nb of payloads)');
    title(str_title);
    
       
    legend_handle = legend([ g2 , p2 , g3 , p3], {'Mean Test', 'Variance test' , 'Mean Extrapol' , 'Var extrapol'}, 'Location','southeast');
    set(gca,'Layer','top')
    xlim([nb_train_data(1),nb_train_data(end)])
end
%}

%}



nr_samples = [];

range=[-2*pi,2*pi];


%% -------------  Preparing data set --------------------------------------------------


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

nb_train_data = [];

jnt_error_train = [];
jnt_error_extrapol_1 = [];
jnt_error_extrapol_2 = [];
var_train_train = [];
var_extrapol_1 = [];
var_extrapol_2 = [];
var_mean_1 = [];
var_mean_2 = [];
var_mean_3 = [];

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

test_error_0 = zeros(10,6);
test_var_0 = zeros(10,6);
test_error_1 = zeros(10,6);
test_var_1 = zeros(10,6);
test_error_3 = zeros(10,6);
test_var_3 = zeros(10,6);
test_error_5 = zeros(10,6);
test_var_5 = zeros(10,6);

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
    extra_data_error_0 = [];
    extra_data_error_1 = [];
    extra_data_error_3 = [];
    extra_data_error_5 = [];


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
    
    
    for idx = 1:size(samples_train_X_0 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(0.0001) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(samples_train_X_0(1:6 , idx )');
        extra_data_error_0 = [extra_data_error_0 (elm_result - samples_train_Y_0(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error_0(i,jntidx)=(sum((extra_data_error_0(jntidx,:)).^2,2))/size(samples_train_X_0 , 2);
        test_var_0(i,jntidx)=var(extra_data_error_0(jntidx,:).^2);
    end
        for idx = 1:size(samples_train_X_1 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(1) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(samples_train_X_1(1:6 , idx )');
        extra_data_error_1 = [extra_data_error_1 (elm_result - samples_train_Y_1(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error_1(i,jntidx)=(sum((extra_data_error_1(jntidx,:)).^2,2))/size(samples_train_X_1 , 2);
        test_var_1(i,jntidx)=var(extra_data_error_1(jntidx,:).^2);
    end
        for idx = 1:size(samples_train_X_3 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(3) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(samples_train_X_3(1:6 , idx )');
        extra_data_error_3 = [extra_data_error_3 (elm_result - samples_train_Y_3(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error_3(i,jntidx)=(sum((extra_data_error_3(jntidx,:)).^2,2))/size(samples_train_X_3 , 2);
        test_var_3(i,jntidx)=var(extra_data_error_3(jntidx,:).^2);
    end
        for idx = 1:size(samples_train_X_5 , 2)
        elm_learner.wOut = reshape(elm_on_elm.apply(5) , [size(elm_learner.wOut)]);
        elm_result = elm_learner.apply(samples_train_X_5(1:6 , idx )');
        extra_data_error_5 = [extra_data_error_5 (elm_result - samples_train_Y_5(:,idx)')'];    
    end
    for jntidx = 1:6
        test_error_5(i,jntidx)=(sum((extra_data_error_5(jntidx,:)).^2,2))/size(samples_train_X_5 , 2);
        test_var_5(i,jntidx)=var(extra_data_error_5(jntidx,:).^2);
    end
    
    
    
end % end of i - loop for testing


    temp_jnt_err = [];
    temp_jnt_var = [];
    temp_jnt_err_train = [];
    temp_jnt_var_train = [];
    temp_jnt_err_extrapol = [];
    temp_jnt_var_extrapol = [];
    var_err = [];
    var_extrapol = [];
    var_train = [];
    for jntidx = 1:6
        temp_jnt_err = [temp_jnt_err mean(test_error(:,jntidx))];
        temp_jnt_var = [temp_jnt_var mean(test_var(:,jntidx))];
        temp_jnt_err_train = [temp_jnt_err_train (mean(test_error_0(:,jntidx)) + mean(test_error_1(:,jntidx)) + mean(test_error_3(:,jntidx)) + mean(test_error_5(:,jntidx)))/4];
        temp_jnt_var_train = [temp_jnt_var_train (mean(test_var_0(:,jntidx)) + mean(test_var_1(:,jntidx)) + mean(test_var_3(:,jntidx)) + mean(test_var_5(:,jntidx)))/4];
        temp_jnt_err_extrapol = [temp_jnt_err_extrapol mean(test_error_extrapol(:,jntidx))];
        temp_jnt_var_extrapol = [temp_jnt_var_extrapol mean(test_var_extrapol(:,jntidx))];
        var_err = [var_err var(test_error(:,jntidx))];
        var_extrapol = [var_extrapol var(test_error_extrapol(:,jntidx))];
        var_train = [var_train (var(test_error_0(:,jntidx)) + var(test_error_1(:,jntidx)) + var(test_error_3(:,jntidx)) + var(test_error_5(:,jntidx)))/4];
    end    

    
    jnt_error_extrapol_1 = [jnt_error_extrapol_1 temp_jnt_err'];
    jnt_error_extrapol_2 = [jnt_error_extrapol_2 temp_jnt_err_extrapol'];
    jnt_error_train = [jnt_error_train temp_jnt_err_train'];
    var_extrapol_1 = [var_extrapol_1 temp_jnt_var'];
    var_extrapol_2 = [var_extrapol_2 temp_jnt_var_extrapol'];
    var_train_train = [var_train_train temp_jnt_var_train'];
    var_mean_1 = [var_mean_1 var_err'];
    var_mean_2 = [var_mean_2 var_extrapol'];
    var_mean_3 = [var_mean_3 var_train'];
    
end
    
    
jnt_error_test = [];
jnt_error_test = jnt_error_extrapol_1;
var_test = var_extrapol_1;
jnt_error_extrapol = jnt_error_extrapol_2;
var_extrapol = var_extrapol_2;
var_train = var_train_train;

    
    
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
