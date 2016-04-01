function [wsProMPsPrior, mu_w, sigma_w, datasets] = training(datasets, lambdaProMPs)

%% Learn a distribution using ProMPs
numGaussians = 10;

numDemonstrations = length(datasets);
[DOF, T] = size(datasets(1).Ymat);

%Linearly distributed basis functions
for l=1:numDemonstrations
    %Note the trajectory length T might change with the demonstrations
    datasets(l).PSI_X = sparse(getBasisFunctionsMultipleJointsAndTimesteps(numGaussians,T,DOF,1));
end

PSI_X = datasets(1).PSI_X;

wsProMPsPrior = ProMPs_train(datasets,size(datasets(1).PSI_X,2), size(datasets(1).Y,2), lambdaProMPs); 
%Learned Prior:
mu_w = mean(wsProMPsPrior,3);
sigma_w = cov(squeeze(wsProMPsPrior(:,1,:))');

end