function resp = imitationCallback(server,req,resp)

    y_start = [req.YStart.X, req.YStart.Y, req.YStart.Z]';
    y_end = [req.YEnd.X, req.YEnd.Y, req.YEnd.Z]';
    [~, ~, resp.Filename] = movementImitation(y_start, y_end);
    resp.Response = 1;

end
