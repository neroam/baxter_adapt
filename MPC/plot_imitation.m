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
        
        xlabel('X (m)', 'fontsize', fs);
        ylabel('Y (m)', 'fontsize', fs);
        zlabel('Z (m)', 'fontsize', fs);
        
        prior_plot=plot3(y_prior(:,1)', y_prior(:,2)', y_prior(:,3)', '-.b','linewidth',lw);
        hold on;
        est_plot=plot3(y_predProMPsMat(1,:), y_predProMPsMat(2,:), y_predProMPsMat(3,:),'-r','linewidth',lw*2);
        hObs=scatter3(y_testMat(1,:), y_testMat(2,:), y_testMat(3,:), 100, 'og', 'linewidth',lw*2);
        lhndLegend=legend([prior_plot,est_plot,hObs], 'Prior Mean Trajectory','Predicted Mean Trajectory','Task Contexts');


        global contexts;
        
        objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4); 
        r = 0.1;
        [x,y,z] = sphere(30);
        x = x*r+contexts.obstacles(1);
        y = y*r+contexts.obstacles(2);
        z = z*r+contexts.obstacles(3);
        surface(x,y,z,'FaceColor','none','EdgeColor',[0.8,0.8,0.8]);
        
        hObs.MarkerEdgeColor = [0, 1,0 ];
        
        lhndLegend=legend([prior_plot,est_plot,hObs,objects], 'Prior Mean Trajectory','Predicted Mean Trajectory','Task Contexts','Obstacle');

        
        
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
