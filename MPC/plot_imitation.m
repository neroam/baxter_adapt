function y_imit = plot_imitation(y_prior,y_predProMPsMat,y_testMat, datasetsTest)

DOF = size(y_prior,2);
fs = 32;
lw = 2;
set(figure,'Color','white');
y_imit = [];

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
    
else if DOF >= 3
        
        for i = 1:size(y_prior,1)
            y_prior(i,:) = forwardKine(y_prior(i,:)')';
        end
        for i = 1:size(y_predProMPsMat,2)
            y_predProMPsMat(:,i) = forwardKine(y_predProMPsMat(:,i));
        end
        for i = 1:size(y_testMat,2)
            y_testMat(:,i) = forwardKine(y_testMat(:,i));
        end
        y_imit = y_predProMPsMat';
        
        prior_plot=plot3(y_prior(:,1)', y_prior(:,2)', y_prior(:,3)', '-.','linewidth',lw);
        hold all;
        est_plot=plot3(y_predProMPsMat(1,:), y_predProMPsMat(2,:), y_predProMPsMat(3,:),'linewidth',lw*2);
        hObs=scatter3(y_testMat(1,:), y_testMat(2,:), y_testMat(3,:), 100, 'xb', 'linewidth',lw*2);
        
        xlabel('X (m)', 'fontsize', fs);
        ylabel('Y (m)', 'fontsize', fs);
        zlabel('Z (m)', 'fontsize', fs);
        
        lhndLegend=legend([prior_plot,est_plot,hObs], 'Prior Mean Trajectory','Predicted Mean Trajectory','Task Contexts');
        
        if nargin == 4
            for i = 1:size(datasetsTest.Ymat,2)
                datasetsTest.Ymat(:,i) = forwardKine(datasetsTest.Ymat(:,i));
            end
        
            
            hTrue=plot3(datasetsTest.Ymat(1,:), datasetsTest.Ymat(2,:),datasetsTest.Ymat(3,:), '--', 'linewidth', lw);
            lhndLegend=legend([prior_plot,est_plot,hTrue,hObs], 'Prior Mean Trajectory','Predicted Mean Trajectory','True Trajectory','Task Contexts');
        end
        set(lhndLegend, 'fontsize', fs);
        set(gca, 'fontsize', fs);    
    end
end
    
end
