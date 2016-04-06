function resp = imitationCallback(server,req,resp)

    [~, ~, resp.Filename] = movementImitation(req.YStart, req.YEnd);
    resp.Response = 1;

end
