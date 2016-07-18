clear all;

%% setup
inpDim = 10;
outDim = 2;
numSamples = 1000;

A = randn(outDim,inpDim);
b = randn(1,outDim);

%% create data
X = randn(numSamples, inpDim);
Y = X * A' + repmat(b, numSamples, 1);

%% create learner
lspecs = {'class', 'ELM'; 'hidDim', 100; 'reg', 1e-6; 'BIP', 0; 'mu', 0.2};
%lspecs = {'class', 'OnlineWeightedLinearLearner'; 'lrate', 0.01;};
%lspecs = {'class', 'LLM'; 'lrate', 0.05; 'radius', 2};
cmd = ['learner = ' lspecs{1,2} '(inpDim, outDim, lspecs);'];
eval(cmd);

%% train
learner.init(X);
learner.train(X, Y);

%% test
Yhat = learner.apply(X);
disp(['RMSE = ' num2str(mean(sqrt(sum((Y - Yhat).^2,2))))]);

