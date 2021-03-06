function [est_plot, y_predVarProMPsMat] = ProMPs_plot_prediction(y_predProMPs, sigma_starProMPs, PSI_X, lambdaProMPs, T, DOF, y_testMat, y_predProMPsMat,datasetsTest)

lw = 2;
fs = 32;
fsLegend=18;

y_predVarProMPs = zeros(size(y_predProMPs));
for t=1:size(y_predProMPs,1)
    y_predVarProMPs(t,:) =  PSI_X(t,:)*sigma_starProMPs*PSI_X(t,:)' + lambdaProMPs;
end
y_predVarProMPsMat = reshape(y_predVarProMPs,DOF,T);
timeIds = linspace(0,5,T);
for i=1:DOF   
    hFigPrediction(i) = figure;
    set(hFigPrediction(i),'Color','white');
    hold all;
    lndPred =  shadedErrorBar(timeIds,y_predProMPsMat(i,:),sqrt(y_predVarProMPsMat(i,:)),'r',1);
    est_plot = lndPred.mainLine;
    set(est_plot, 'linewidth', 3);
    hObs=scatter(timeIds([1 T])', y_testMat(i,:)',100, 'xb', 'linewidth',lw*2);
           
    xlabel('Time (s)', 'fontsize', fs);
    ylabel(['Joint ' num2str(i) ' (rad)'] , 'fontsize', fs);
    lhndLegend=legend([est_plot, hObs], 'Predicted Mean Trajectory', 'Task Contexts');
    
    if nargin == 9
        hTrue=plot(timeIds, datasetsTest.Ymat(i,:), '--', 'linewidth', lw);
        lhndLegend=legend([est_plot, hTrue, hObs], 'Predicted Mean Trajectory', 'True Trajectory', 'Task Contexts');
    end
    set(lhndLegend, 'fontsize',fs);
    set(gca, 'fontsize', fs);
end

