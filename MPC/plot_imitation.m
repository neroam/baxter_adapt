function plot_imitation(y_prior,y_predProMPsMat,y_testMat)

DOF = size(y_prior,2);
fs = 26;
lw = 2;
set(figure,'Color','white');

if DOF == 2
    
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
    
else if DOF == 3
        
        prior_plot=plot3(y_prior(:,1)', y_prior(:,2)', y_prior(:,3)', '--','linewidth',lw*2);
        hold all;
        est_plot=plot3(y_predProMPsMat(1,:), y_predProMPsMat(2,:), y_predProMPsMat(3,:),'linewidth',lw*2);
        % hTrue=plot(datasetsTestTmp.Ymat(1,:), datasetsTestTmp.Ymat(2,:), 'linewidth', lw);
        hObs=scatter3(y_testMat(1,:), y_testMat(2,:), y_testMat(3,:), 100, 'xb', 'linewidth',lw*2);
        
        xlabel('x', 'fontsize', fs);
        ylabel('y', 'fontsize', fs);
        zlabel('z', 'fontsize', fs);
            
    end
end
    
lhndLegend=legend([prior_plot, est_plot,hObs], 'Prior Mean Trajectory','Predicted Mean Trajectory','Task Contexts');
set(lhndLegend, 'fontsize', fs);
set(gca, 'fontsize', fs);

end
