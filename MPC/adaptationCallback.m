function resp = adaptationCallback(server,req,resp)

    req
    
    y_start = [req.YStart.X, req.YStart.Y, req.YStart.Z]';
    y_end = [req.YEnd.X, req.YEnd.Y, req.YEnd.Z]';

    obstacles = [];
    num_obst = length(req.Obstacles);
    for i = 1:num_obst
       obstacles = [obstacles, [req.Obstacles(i).X, req.Obstacles(i).Y, req.Obstacles(i).Z]']; 
    end

    contexts.obstacles = obstacles

    num_weights = 4 + 5*num_obst + 2; 

    global weights;

    if exist('weights','var') == 0 || num_weights ~= length(weights)
        display('Initialzie weights');
        weights = [10 10 10 10, zeros(1,5*num_obst), 0 0]';
    end

    weights
    
    resp.Filename = [pwd,'/generated/adaptationTrajectory'];
    delete(resp.Filename);
    display('Cache deleted');
    
    F=parfeval(@movementAdaptation,1,y_start, y_end, weights, contexts)

    resp.Response = 1;

end
