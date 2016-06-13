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

samples_X = [];
samples_Y = [];



[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/fixed_positions/test_data_no_mass_set1.txt');
samples_X = cat(3, samples_X ,samples_X_temp);
samples_Y = cat(3,samples_Y ,samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];
[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/fixed_positions/test_data_mass1.txt');
samples_X = cat(3,samples_X, samples_X_temp);
samples_Y = cat(3,samples_Y ,samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];
[samples_X_temp  , samples_Y_temp , nr_samples_temp] = readData('/homes/abalayn/workspace/rtt-gazebo-ur5-integration/recorded_data/fixed_positions/test_data_mass5.txt');
samples_X = cat(3,samples_X, samples_X_temp);
samples_Y = cat(3,samples_Y, samples_Y_temp);
nr_samples = [nr_samples nr_samples_temp];
% --------------------------------------------------------------------------------------------------------





% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------

% ------------- Parameters optimization -----------------------------------------

% --------------------------------------------------------------------------------------------------------

% --------------------------------------------------------------------------------------------------------


regularization=[ 1e-11 1e-10 1e-9 1e-8 1e-7 1e-6 1e-5 1e-4 1e-3 1e-2 1e-1 1 10];
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
hiddim = 100;
reg = 1e-5;


massidx = 1;


bipidx = 0;
%for bipidx = BIP
   % for fw = fermiwidth
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
elm_learner=ELM(6,6,{'BIP',bipidx;'reg', reg; 'hidDim', hiddim});






% ------------- Preparing train and test data (cross validation) -----------------------------------------


% Number of data for each set
nb_fold = 5;
c = cvpartition(nr_samples(1),'KFold' , nb_fold);
errors_test=zeros(nb_fold,1);
errors_train=zeros(nb_fold,1);
for idx = 2:6
    cat(3,errors_test,zeros(nb_fold,1));
    cat(3,errors_train,zeros(nb_fold,1));
end

nr_train_s = c.TrainSize;
nr_test_s = c.TestSize;

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

        for jntidx = 1:6
            resultmatrix_test(idx_reg,idx_hiddim , jntidx)=mean(errors_test(:,jntidx));
            resultmatrix_var_test(idx_reg,idx_hiddim , jntidx)=var(errors_test(:,jntidx));
            resultmatrix_train(idx_reg,idx_hiddim, jntidx)=mean(errors_train(:,jntidx));
            resultmatrix_var_train(idx_reg,idx_hiddim , jntidx)=var(errors_train(:,jntidx));
        end    

	end
	end



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

%-------------------------------------------------------------------------------------------

% Plot torque as function of angle. massidx must be specified.

str_title=sprintf('Torques as function of angles');
figure_saver2 = figure('Name',str_title,'NumberTitle','on');

subplot(2,3,1);
str_title=sprintf('Torque as function of angle 0');
plot(samples_X(1,: , massidx),samples_Y(1,: , massidx) , '*', samples_X(1,: , 2),samples_Y(1,: , 2) , '*');
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
%-------------------------------------------------------------------------------------------



% --------------------------------------------------------------------------------------------------



% ------------- Writing parameters in file ---------------------------------------------------------


%Save ELM to file for import in c++  (maybe move into elm class ?!):

target = '../elmmodel';
datatarget = sprintf('%s/data/',target);

mkdir(target);
mkdir(datatarget);

%Write matrix data
%Ax = diag(2./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:)));
Ax = diag( 1./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:)));
bx = elm_learner.inpOffset(1,:)+elm_learner.inpRange(1, :);
%bx = -(2./(elm_learner.inpRange(2,:)-elm_learner.inpRange(1,:))).*(elm_learner.inpOffset + elm_learner.inpRange(1,:)) - 1;
%Ay = diag(2./(elm_learner.outRange(2,:)-elm_learner.outRange(1,:)));
Ay = diag(elm_learner.outRange(2,:)-elm_learner.outRange(1,:));
%by = -(2./(elm_learner.outRange(2,:)-elm_learner.outRange(1,:))).*(elm_learner.outOffset + elm_learner.outRange(1,:)) - 1;
by = elm_learner.outOffset(1,:)+elm_learner.outRange(1, :);
str = sprintf('%s/Win.mat',datatarget);
funclocal_Matrix2TXT(elm_learner.wInp, str);
str = sprintf('%s/a.vec',datatarget);
funclocal_Matrix2TXT(elm_learner.a', str);
str = sprintf('%s/b.vec',datatarget);
funclocal_Matrix2TXT(elm_learner.b', str);
str = sprintf('%s/Wout.mat',datatarget);
funclocal_Matrix2TXT(elm_learner.wOut', str, true);
str = sprintf('%s/Ax.mat',datatarget);
funclocal_Matrix2TXT(Ax, str, true);
str = sprintf('%s/bx.vec',datatarget);
funclocal_Matrix2TXT(bx, str);
str = sprintf('%s/Ay.mat',datatarget);
funclocal_Matrix2TXT(Ay, str,true);
str = sprintf('%s/by.vec',datatarget);
funclocal_Matrix2TXT(by, str);

%Write matrix format
str = sprintf('%s/info',target);
fid = fopen(str,'wt');
fprintf(fid, 'elm+datascale');
fclose(fid);

