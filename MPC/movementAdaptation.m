function y_adapted=movementAdaptation(y_start, y_end, weights, contexts)

[y_predVar, y_predProMPsMat]=movementImitation(y_start, y_end);

%% Offline Learned trajectory
refTraj = y_predProMPsMat';
refVar = y_predVar';

%% Configuration
config.horizon = 29;
config.step = 29;
config.umax = 0.2;
config.dmin = 0.01;
config.scale = 1;
config.alpha = 1;
config.dboarder = 0.01;
fs = 26;
lw = 2;

%% Contexts initialization
contexts.refTraj = refTraj;
contexts.refVar = refVar;
contexts.boarder = [-0.2 1.1];
contexts.ground = -0.2;

%contexts.obstacles = [0.84, 0.29, -0.02]';

%% Generate adapted trajectory
[y_adapted, ~] = generateTraj(contexts, weights, config);

filename = [pwd,'/generated/adaptationTrajectory'];
writeTrajectory(filename, y_adapted');

    
%% Plot adapted trajectory
figure;
imitation=plot3(refTraj(:,1)', refTraj(:,2)', refTraj(:,3)' ,'--','linewidth',lw*2);
hold on;
adapt=plot3(y_adapted(:,1)', y_adapted(:,2)', y_adapted(:,3)','linewidth',lw*2);
objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
% object2=scatter3(contexts.obstacles(1,2),contexts.obstacles(2,2),contexts.obstacles(3,1), 100, 'ob', 'linewidth',4);
lhndLegend=legend([imitation, adapt,objects], 'Imitation Trajectory','Adaptation Trajectory','Obstacles');
set(lhndLegend, 'fontsize', fs);
set(gca, 'fontsize', fs);


end
