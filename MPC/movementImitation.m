function [y_predVar, y_predProMPsMat, filename]=movementImitation(y_start, y_end)

close all;

if exist('loaded','var') == 0
    load('trainedData_joints2.mat');
end

[y_predProMPs, y_predProMPsMat, sigma_starProMPs, y_testMat] = testing(PSI_X, y_start, y_end, wsProMPsPrior,lambdaProMPs, T, DOF);

filename = [pwd,'/generated/imitationTrajectory'];

writeTrajectory(filename, y_predProMPsMat);

y_predVar = reshape(y_predProMPsMat, DOF, T);
% %% Plot the predictive distribution 
% ProMPs_plot_prediction(y_predProMPs, sigma_starProMPs, PSI_X, ...
%     lambdaProMPs, T, DOF, y_testMat, y_predProMPsMat);


%% Plot the imitation trajectory
plot_imitation(y_prior,y_predProMPsMat,y_testMat)

end
