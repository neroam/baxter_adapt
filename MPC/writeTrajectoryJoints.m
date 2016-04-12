function writeTrajectoryJoints(filename, y)

fid = fopen(filename, 'w');
fprintf(fid, 'time,left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2\n');
fclose(fid);
time_stamp = linspace(0, 2, size(y,2));
y_stamped = [time_stamp; y]';

dlmwrite(filename, y_stamped, '-append', 'precision', '%.6f', 'delimiter', ',');

end