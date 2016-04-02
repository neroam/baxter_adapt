function y_adapted=movementAdaptation(y_start, y_end, weights)

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

contexts.obstacles = [0.84 0.31, 0]';

%% Generate adapted trajectory
[y_adapted, ~] = generateTraj(contexts, weights, config);
    
%% Plot adapted trajectory
figure;
plot3(refTraj(:,1)', refTraj(:,2)', refTraj(:,3)');
hold on;
plot3(y_adapted(:,1)', y_adapted(:,2)', y_adapted(:,3)');
scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:), contexts.obstacles(3,:),100, 'xb', 'linewidth',4);
plot3(contexts.obstacles(1,:),contexts.obstacles(2,:), contexts.obstacles(3,:));


end