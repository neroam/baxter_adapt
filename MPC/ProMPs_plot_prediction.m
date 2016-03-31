function [est_plot, y_predVarProMPsMat] = ProMPs_plot_prediction(datasetsTest, lTestId, y_predProMPs, sigma_starProMPs, lambdaProMPs, T, DOF, usedSampleIds, y_testMat, y_predProMPsMat)

lw = 2;
fs = 26;
fsLegend=18;

datasetsTestTmp = datasetsTest(lTestId);
y_predVarProMPs = zeros(size(y_predProMPs));
for t=1:size(y_predProMPs,1)
    y_predVarProMPs(t,:) =  datasetsTestTmp.PSI_X(t,:)*sigma_starProMPs*datasetsTestTmp.PSI_X(t,:)' + lambdaProMPs;
end
y_predVarProMPsMat = reshape(y_predVarProMPs,DOF,T);
timeIds = linspace(0,1,T);
for i=1:DOF   
    hFigPrediction(i) = figure;
    set(hFigPrediction(i),'Color','white');
    hold all;
    lndPred =  shadedErrorBar(timeIds,y_predProMPsMat(i,:),sqrt(y_predVarProMPsMat(i,:)),'r',1);
    est_plot = lndPred.mainLine;
    set(est_plot, 'linewidth', 3);
%     hTrue=plot(timeIds, datasetsTestTmp.Ymat(i,:), 'k', 'linewidth', lw);
    hObs=scatter(timeIds(usedSampleIds)', y_testMat(i,:)',100, 'xb', 'linewidth',lw*2);
           
    xlabel('Movement Phase', 'fontsize', fs);
    ylabel(['Dimension ' num2str(i)] , 'fontsize', fs);
    
    lhndLegend=legend([est_plot, hObs], 'Predicted Mean Trajectory', 'Task Contexts');
    set(lhndLegend, 'fontsize',fs);
    set(gca, 'fontsize', fs);
end

