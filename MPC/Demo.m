clear all;
close all;
clc;
    
synthetic = 0;
datasets = prepareData(synthetic);
% datasets = datasets(1);
[DOF,T] = size(datasets(1).Ymat);
lambdaProMPs = 1e-5; %regularization parameter to avoid singularities

%% Learn a distribution using ProMPs
[wsProMPsPrior, mu_w, sigma_w, datasets] = training(datasets,lambdaProMPs);
PSI_X = datasets(1).PSI_X;
    
%% Plot the prior distribution
flgPlotData = 0;
[~,y_prior]=ProMPs_plot_prior(datasets, wsProMPsPrior, lambdaProMPs, flgPlotData, DOF);


%% Predict future trajectory points given some observations
% for simplicity, one of the training traj. is used
datasetsTest = datasets;
lTestId = 1;

y_start = datasets(lTestId).Ymat(:,1);
y_end = datasets(lTestId).Ymat(:,T);

[y_predProMPs, y_predProMPsMat, sigma_starProMPs, y_testMat] = testing(PSI_X, y_start, y_end, wsProMPsPrior,lambdaProMPs, T, DOF);

%% Plot the predictive distribution 
[~, y_predVar] = ProMPs_plot_prediction(y_predProMPs, sigma_starProMPs, PSI_X, ...
    lambdaProMPs, T, DOF, y_testMat, y_predProMPsMat,datasetsTest(lTestId));

%% Plot the imitation trajectory
% plot_imitation(y_prior,y_predProMPsMat,y_testMat);
plot_imitation(y_prior,y_predProMPsMat,y_testMat,datasetsTest(lTestId));


%% Save trained data
loaded = 1;
save('trainedData_joints_all.mat', 'DOF','lambdaProMPs','PSI_X','T','wsProMPsPrior','y_prior','loaded');




