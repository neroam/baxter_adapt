function writeTrajectory(filename, y)

fid = fopen(filename, 'w');
fprintf(fid, 'time,left_pos_x,left_pos_y,left_pos_z\n');
fclose(fid);
time_stamp = linspace(0, 2, size(y,2));
y_stamped = [time_stamp; y]';

dlmwrite(filename, y_stamped, '-append', 'precision', '%.6f', 'delimiter', ',');

end