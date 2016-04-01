function writeTrajectory(filename, y)

fid = fopen(filename, 'w');
fprintf(fid, 'left_pos_x,left_pos_y,left_pos_z\n');
fclose(fid);

dlmwrite(filename, y', '-append', 'precision', '%.6f', 'delimiter', ',');

end