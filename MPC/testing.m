function [y_predProMPs, y_predProMPsMat, sigma_starProMPs, y_testMat] = testing(PSI_X, y_start, y_end, wsProMPsPrior, lambdaProMPs, T, DOF)

% [DOF, T] = size(datasets(1).Ymat);
% PSI_X = datasets(1).PSI_X;

lambdaProMPsPrediction = lambdaProMPs*ones(1,DOF); %only condition on the first DoF

X_test = [PSI_X(1:DOF,:) ;PSI_X((T-1)*DOF+1:T*DOF,:)];
y_test = [y_start ;y_end];

[w_starProMPs, sigma_starProMPs] = ProMPs_prediction(X_test,y_test,wsProMPsPrior,lambdaProMPsPrediction,DOF); 
y_predProMPs = PSI_X * w_starProMPs;
y_predProMPsMat = reshape(y_predProMPs,DOF,T);
y_testMat = reshape(y_test, DOF, length(y_test)/DOF);
