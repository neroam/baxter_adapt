close all;

joints_dim = 7;

contexts.start_joints = [-0.37275733103,-0.415325297845,0.192131093463,1.27051958611,-0.23009711792,0.749733109222,-1.41164581844]';
contexts.obstacles = [0.676, 0.49, 0.04]';

target = [-1.34836911101,-0.519252496106,0.24888838255,1.58958758963,-0.441019476013,0.488572880383,-2.19934495212]';

num_obst = size(contexts.obstacles,2);
weights = [10+zeros(1, joints_dim), 10, zeros(1,5*num_obst), 0, 0]';

M = 10;

err = zeros(1,M);
err_joints = zeros(1,M);

[y_predVar, y_predProMPsMat]=movementImitation(contexts.start_joints, target);

refTraj = y_predProMPsMat';
refVar = y_predVar';

%% Configuration
config.horizon = 11;
config.step = 11;
config.umax = 2;
config.dmin = 0.01;
config.scale = 1;
config.alpha = 1;
config.dboarder = 0.01;
fs = 26;
lw = 2;

%% Contexts initialization
contexts.refTraj = refTraj;
contexts.refVar = refVar;
contexts.boarder = [0.8 -0.2];
contexts.ground = -0.2;
 
 for i = 1:size(refTraj,1)
     y_imitated(i,:) = forwardKine(refTraj(i,:)')';
 end

for k = 1:M
    
    [y_adapted, adapted_joints, ~] = jointsTraj(contexts, weights, config);
    
    err(k) = sum(sum((y_adapted-y_improved).^2,1));
    err_joints(k) = sum(sum((adapted_joints-improved_joints).^2,1));
      
    %% Plot adapted trajectory
    figure;
    imitation=plot3(y_imitated(:,1)', y_imitated(:,2)', y_imitated(:,3)' ,'--','linewidth',lw*2);
    hold on;
    adapt=plot3(y_adapted(:,1)', y_adapted(:,2)', y_adapted(:,3)','linewidth',lw*2);
    objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
    % object2=scatter3(contexts.obstacles(1,2),contexts.obstacles(2,2),contexts.obstacles(3,1), 100, 'ob', 'linewidth',4);
    lhndLegend=legend([imitation, adapt,objects], 'Imitation Trajectory','Adaptation Trajectory','Obstacles');
    set(lhndLegend, 'fontsize', fs);
    set(gca, 'fontsize', fs);
    
    w_improved = learningWeights(adapted_joints, improved_joints, contexts, weights, config);
    
    fprintf('Finished iteration %d\n',k);
    weights = w_improved
    
end

figure;
plot(err,'MarkerFaceColor','black','Marker','s','linewidth',lw*2);
grid on;
xlabel('Learning iterations', 'fontsize', fs);
ylabel('Learning error', 'fontsize', fs);
set(gca, 'fontsize', fs);


figure;
plot(err_joints,'MarkerFaceColor','black','Marker','s','linewidth',lw*2);
grid on;
xlabel('Learning iterations', 'fontsize', fs);
ylabel('Learning error', 'fontsize', fs);
set(gca, 'fontsize', fs);
