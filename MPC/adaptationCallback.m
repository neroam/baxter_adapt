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
    
    num_obst = length(req.Obstacles);    
    contexts.obstacles = zeros(3, num_obst);    
    for i = 1:num_obst
        contexts.obstacles(:,i) = [req.Obstacles(i).X, req.Obstacles(i).Y, req.Obstacles(i).Z]'; 
    end
 
    %%% Movement Imitation
    [y_predVar, y_predProMPsMat, y_imit]=movementImitation(target, contexts);
    
    joints_imit = y_predProMPsMat';
    
   
    %%% Movement Adaptation
    if req.EnableAdapt
        delete(resp.Filename);
        display('Cache deleted');
        joints_adapt = [];
        y_adapt = [];

        num_weights = 8 + 5*num_obst + 2; 

        if exist('weights','var') == 0 || num_weights ~= length(weights)
            display('Initialzie weights');
            weights = [zeros(1,7)+30,30, zeros(1,5*num_obst), 0, 0]';
        end

        weights

        %% Contexts initialization
        contexts.refTraj = y_predProMPsMat';
        contexts.refVar = y_predVar';
    
        parfeval(@jointsTraj,1,contexts, weights, config);
    else
        filename = contexts.filename;
        writeTrajectoryJoints(filename, y_predProMPsMat);
    end
        
       
    resp.Response = 1;
end
