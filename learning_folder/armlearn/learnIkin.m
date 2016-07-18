clear all;
close all;

%restoredefaultpath;
addpath('../learner/algorithms');
addpath('../learner/metric');
addpath('../learner/utils');


arm=TwoDOFManipulator();

%create dataset
range=[-pi,pi];
len = range(2)-range(1);
nr_samples_train=40;  %only 40 samples for training ! 
nr_samples_test=1000;

samples_train_X=rand(2,nr_samples_train)*len+range(1);


samples_test_X=rand(2,nr_samples_test)*len+range(1);


samples_train_Y=arm.fkin(samples_train_X);
samples_test_Y=arm.fkin(samples_test_X);


[xq,yq] = meshgrid(range(1):.1:range(2), range(1):.1:range(2));

testdata=[reshape(xq,1,size(xq,1)*size(xq,2));reshape(yq,1,size(yq,1)*size(yq,2))];
testresult=arm.fkin(testdata);
outq=reshape(testresult(2,:),size(xq,1),size(xq,2));


figure(10)

mesh(xq,yq,outq);
caxis([-1 1]);
title('Orig data dense')
view([0 90]);
rotate3d;


%linear triangle:
figure(1);
lininterpol_result = griddata(samples_train_X(1,:),samples_train_X(2,:), samples_train_Y(2,:),xq,yq);
mesh(xq,yq,lininterpol_result);
caxis([-1 1])
title('linear interpolation')
view([0 90]);
rotate3d;
%testresult_linear=reshape(lininterpol_result,1,size(xq,1)*size(xq,2));

outq(isnan(lininterpol_result))=NaN;
err_normalization=sum(sum(~isnan(outq)));
%linerror=nansum(nansum((outq-lininterpol_result).^2))./err_normalization;




%elm:
%BIP - Batch intrinsic pasticity (extension for tuning of parameters)
%reg - regularization for linear output learning
%hidDim - hidden layer size (random layer)
%
%
elm_learner=ELM(2,1,{'BIP',0;'reg', 1*1e-10; 'hidDim',50});

%scale inputs to range etc:
elm_learner.init(samples_train_X');
%elm_learner.b sind die centers der fermi funktionen - alle random
%elm_learner.a 'width' of fermi function, default: 1
%the lager 'a' gets, the 'smaller' the fermi function


%elm_learner.a=0.01.*elm_learner.a; % -> extremely wide fermi function
%almost linear

%elm_learner.a=5.*elm_learner.a; % -> smaller sigmoid in function space


%train the stuff
elm_learner.train(samples_train_X', samples_train_Y(2,:)');

%execute on testdata
testdata = [reshape(xq,size(xq,1)*size(xq,2),1),reshape(yq,size(yq,1)*size(yq,2),1)];
[elm_result] = elm_learner.apply(testdata);

%calc performance
elm_result=reshape(elm_result,size(xq,1),size(xq,2));
figure(2);
mesh(xq,yq,elm_result);
caxis([-1 1])
title('elm result')
view([0 90]);
rotate3d;


%elmerror=nansum(nansum((outq-elm_result).^2))./err_normalization;
figure(3)
mesh(xq,yq,abs(elm_result-outq));
title('elm errors + triandata points')
view([0 90]);
rotate3d;
hold on;
plot3(samples_train_X(1,:), samples_train_X(2,:), samples_train_Y(2,:).*0, 'rx');

%result:
%['linear interpolation error' num2str(linerror)]
%['elm error' num2str(elmerror)]


tilefigs()







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

 
%eval to compare to c++ version:
vmax=3;
vmin=-3;
for t=0:100
    val =((vmax-vmin)*t/100.0) + vmin;
    evaldata = [1 1].*val ;
    res = elm_learner.apply(evaldata);
    
    disp(['eval ' num2str(evaldata) ' -> ' num2str(res)]);
    
    
end

