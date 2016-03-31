clear all;
close all;
clc;
    
simulation_3D = 0;

%% Create synthetic data
numDemonstrations = 50;
DOF = 2;

datasets = struct([]);

T = 500;
usedSampleStep = T-1;                           %every 10th point is observed
t=zeros(DOF,T);
for i=1:DOF
    t(i,:) = linspace(0,i*0.25*pi,T);
end

hFigRawData=figure; hold all;
set(hFigRawData,'Color','white');
for l=1:numDemonstrations
    data = (1+ 0.3*randn(1,1))*sin(t) + (0.3+ 0.1*randn(1,1))*sin(3*t) + 0.1*randn(DOF,T);
    datasets(l).Y = data(:); %encoding: [val1DoF1, val1DoF2, val1DoF3, ..., val2DoF1, ...]
    datasets(l).Ymat = data; %for plotting only
    plot(data(1,:));
end
xlabel('time step', 'fontsize',26);
ylabel('raw data of joint 1', 'fontsize',26);
set(gca, 'fontsize',26);


%% Learn a distribution using ProMPs
numGaussians = 10;
lambdaProMPs = 1e-5; %regularization parameter to avoid singularities

%Linearly distributed basis functions
for l=1:numDemonstrations
    %Note the trajectory length T might change with the demonstrations
    datasets(l).PSI_X = sparse(getBasisFunctionsMultipleJointsAndTimesteps(numGaussians,T,DOF,1));
end

wsProMPsPrior = ProMPs_train(datasets,size(datasets(1).PSI_X,2), size(datasets(1).Y,2), lambdaProMPs); 
%Learned Prior:
mu_w = mean(wsProMPsPrior,3);
sigma_w = cov(squeeze(wsProMPsPrior(:,1,:))');
    

%% Plot the prior distribution
flgPlotData = 0;
[~,y_prior]=ProMPs_plot_prior(datasets, wsProMPsPrior, lambdaProMPs, flgPlotData, DOF);


%% Predict future trajectory points given some observations
% for simplicity, one of the training traj. is used
datasetsTest = datasets;
lTestId = 1;
usedSampleIds = 1:usedSampleStep:round(T); %first 20%   
% lambdaProMPsPrediction = [lambdaProMPs 1e+5*ones(1,DOF-1)]; %only condition on the first DoF
lambdaProMPsPrediction = lambdaProMPs*ones(1,DOF); %only condition on the first DoF


X_test = zeros(length(usedSampleIds)*DOF,size(datasetsTest(lTestId).PSI_X,2)); 
y_test = zeros(length(usedSampleIds)*DOF,size(datasetsTest(lTestId).Y,2));
for i=1:length(usedSampleIds)
   indices = DOF * (usedSampleIds(i)-1)+1: DOF*usedSampleIds(i);
   X_test((i-1)*DOF + 1:i*DOF,:) = datasetsTest(lTestId).PSI_X(indices,:);
   y_test((i-1)*DOF + 1:i*DOF,:) = datasetsTest(lTestId).Y(indices,:);
end

[w_starProMPs, sigma_starProMPs] = ProMPs_prediction(X_test,y_test,wsProMPsPrior,lambdaProMPsPrediction,DOF); 
y_predProMPs = datasetsTest(lTestId).PSI_X * w_starProMPs;
y_predProMPsMat = reshape(y_predProMPs,DOF,T);
y_testMat = reshape(y_test, DOF, length(y_test)/DOF);



%% Plot the predictive distribution 
[~, y_predVar] = ProMPs_plot_prediction(datasetsTest, lTestId, y_predProMPs, sigma_starProMPs, ...
    lambdaProMPs, T, DOF, usedSampleIds, y_testMat, y_predProMPsMat);

if ~simulation_3D
    
    fs = 26;
    lw = 2;
    set(figure,'Color','white');
    prior_plot=plot(y_prior(:,1)', y_prior(:,2)', '--', 'linewidth',lw*2);
    hold all;
    est_plot=plot(y_predProMPsMat(1,:), y_predProMPsMat(2,:), 'linewidth',lw*2);
    % hTrue=plot(datasetsTestTmp.Ymat(1,:), datasetsTestTmp.Ymat(2,:), 'linewidth', lw);
    hObs=scatter(y_testMat(1,:), y_testMat(2,:),100, 'xb', 'linewidth',lw*2);
    
    xlabel('Dimension 1', 'fontsize', fs);
    ylabel('Dimension 2', 'fontsize', fs);
    
    lhndLegend=legend([prior_plot, est_plot,hObs], 'Prior Mean Trajectory','Predicted Mean Trajectory','Task Contexts');
    set(lhndLegend, 'fontsize', fs);
    set(gca, 'fontsize', fs);
    
else
    fs = 26;
    lw = 2;
    set(figure,'Color','white');
    prior_plot=plot3(y_prior(:,1)', y_prior(:,2)', y_prior(:,3)');
    hold all;
    est_plot=plot3(y_predProMPsMat(1,:), y_predProMPsMat(2,:), y_predProMPsMat(3,:));
    % hTrue=plot(datasetsTestTmp.Ymat(1,:), datasetsTestTmp.Ymat(2,:), 'linewidth', lw);
    hObs=scatter3(y_testMat(1,:), y_testMat(2,:), y_testMat(3,:), 100, 'xb', 'linewidth',lw*2);
    
    xlabel('x', 'fontsize', fs);
    ylabel('y', 'fontsize', fs);
    zlabel('z', 'fontsize', fs);
    
    lhndLegend=legend([prior_plot, est_plot,hObs], 'prior','prediction','observations');
    set(lhndLegend, 'fontsize', fs);
    set(gca, 'fontsize', fs);
    
end





