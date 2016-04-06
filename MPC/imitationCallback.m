function resp = imitationCallback(server,req,resp)
    resp.Filename = 'imitationTrajectory';
    
    movementImitation(req.YStart, req.YEnd);
    
    resp.Response = 1;

end