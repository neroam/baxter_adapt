function resp = adaptationCallback(server,req,resp)

    req
    
    global weights;
    global contexts;
    global config;
    global joints_imit;
    global joints_adapt;
    global y_imit;
    global y_adapt;
    global target;
    
    resp.Filename = [pwd,'/generated/JointsTrajectory'];
    delete(resp.Filename);
    display('Cache deleted');
    contexts.filename = resp.Filename;
    
    %%% Process request
    contexts.start_joints = reshape(req.StartJoints(1:7),7,1);
    target = reshape(req.EndJoints(1:7),7,1);
 
    %%% Movement Imitation
    [y_predVar, y_predProMPsMat]=movementImitation(target, contexts);
    
    joints_imit = y_predVar';
    for i = 1:size(joints_imit,1)
        y_imit(i,:) = forwardKine(joints_imit(i,:)')';
    end
    
    joints_adapt = joints_imit;
    y_adapt = y_imit;
   
    %%% Movement Adaptation
    if req.EnableAdapt
        
        refTraj = y_predProMPsMat';
        refVar = y_predVar';

        num_obst = length(req.Obstacles);
        contexts.obstacles = zeros(3, num_obst);    
        for i = 1:num_obst
           contexts.obstacles(:,i) = [req.Obstacles(i).X, req.Obstacles(i).Y, req.Obstacles(i).Z]'; 
        end

        num_weights = 8 + 5*num_obst + 2; 

        if exist('weights','var') == 0 || num_weights ~= length(weights)
            display('Initialzie weights');
            weights = [zeros(1,7),0, zeros(1,5*num_obst), 0, 0]';
        end

        weights

        %% Contexts initialization
        contexts.refTraj = refTraj;
        contexts.refVar = refVar;
    
        parfeval(@jointsTraj,1,contexts, weights, config);
    end
       
    resp.Response = 1;
end
