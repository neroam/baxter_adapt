function resp = learningCallback(server,req,resp)

    req
    
    global config;
    global contexts;
    global weights;
    global imitation_traj;
    global improved_traj;
    global adaptation_traj;
    
    data = csvread(req.Filename, 1, 0);
    improved_traj = preprocess(data(:,2:4)');
    improved_traj = improved_traj(:,round(linspace(1,size(improved_traj,2),size(adaptation_traj,1))))';

    weights = updateWeights(adaptation_traj, improved_traj, contexts, weights, config);
    
    resp.Filename = [pwd,'/generated/improvedTrajectory'];
    delete(resp.Filename);
    display('Cache deleted');
    
    resp.Response = 1;
    
    figure;
    fs = 26;
    lw = 4;
    imitation = plot3(imitation_traj(:,1)', imitation_traj(:,2)',imitation_traj(:,3)','-.','linewidth',lw*2);
    hold on;
    adapt = plot3(adaptation_traj(:,1)', adaptation_traj(:,2)', adaptation_traj(:,3)','linewidth',lw*2);
    improved = plot3(improved_traj(:,1)', improved_traj(:,2)', improved_traj(:,3)','--','linewidth',lw*2);

    objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
    lhndLegend=legend([imitation, adapt,improved, objects], 'Imitation Trajectory','Adaptation Trajectory','Improved Trajectory','Obstacles');
    set(lhndLegend, 'fontsize', fs);
    set(gca, 'fontsize', fs);


end
