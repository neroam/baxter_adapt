function resp = learningCallback(server,req,resp)

    req
    
    global weights;
    global contexts;
    global config;
    global joints_adapt;
    global joints_improved;
    global y_imit;
    global y_adapt;
    global y_improved;
    
    data = csvread(req.Filename, 1, 0);
    joints_improved= preprocess(data(:,9:15)');
    joints_improved = joints_improved(:,round(linspace(1,size(joints_improved,2),size(joints_adapt,1))))';
    
    for i = 1:size(joints_improved,1)
        y_improved(i,:) = forwardKine(joints_improved(i,:)')';
    end

    weights = learningWeights(joints_adapt, joints_improved, contexts, weights, config);
    
%     resp.Filename = contexts.filename;
%     delete(resp.Filename);
%     display('Cache deleted');
    
    resp.Response = 1;
    
    figure;
    fs = 26;
    lw = 4;
    imitation = plot3(y_imit(:,1)', y_imit(:,2)',y_imit(:,3)','-.','linewidth',lw*2);
    hold on;
    adapt = plot3(y_adapt(:,1)', y_adapt(:,2)', y_adapt(:,3)','linewidth',lw*2);
    improved = plot3(y_improved(:,1)', y_improved(:,2)', y_improved(:,3)','--','linewidth',lw*2);

    objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
    lhndLegend=legend([imitation, adapt,improved, objects], 'Imitation Trajectory','Adaptation Trajectory','Improved Trajectory','Obstacles');
    set(lhndLegend, 'fontsize', fs);
    set(gca, 'fontsize', fs);


end
