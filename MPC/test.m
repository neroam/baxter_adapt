close all;

simulation_3D = 0;

%% Offline Learned trajectory
refTraj = y_predProMPsMat';
refVar = y_predVar';

%% Configuration
N = 1;
M = 10;
err_dist = zeros(M+1,N);
k = 1;
n_o = 2;
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

%%% 2D simulation
if ~simulation_3D
    %% Weights initialization
    weights = [10 10 10 0 0 0 0 0 0 0 0 0 0]';
    % weights = [0.2 0.2 1 0 -0.28 0.28]';
    % weights = [10 10 0 0 1 1];

else     %%% 3D simulation
    %% Weights initialization
    weights = [10 10 10 10 0 0 0 0 1 0 0 0 0 1 0 0]';
    % weights = [0.2 0.2 1 0 -0.28 0.28]';
    % weights = [10 10 0 0 1 1];
end


for i = 1:N
    if ~simulation_3D
%         contexts.obstacles = rand(2,2);
        contexts.obstacles(:,1) = [0.09, 0.20]';
        contexts.obstacles(:,2) = [0.26, 0.47]';
        
%         contexts.obstacles(:,1) = [0.23 0.44]';
%         contexts.obstacles(:,2) = [0.08 0.04]';
       
%         contexts.obstacles(:,1) = [0.72 0.79]';
%         contexts.obstacles(:,2) = [0.835 0.417]';
% 
%         contexts.obstacles(:,1) = [0.33 0.60]';
%         contexts.obstacles(:,2) = [0.09 0.10]';
%         
%     contexts.obstacles = [1.08 1.043]';x`
%     contexts.obstacles(:,2) = rand(2,1);

        set(figure,'Color','white');
        hold all;
        imitation=plot(refTraj(:,1)', refTraj(:,2)', '-.', 'linewidth',lw*2);
        object1=scatter(contexts.obstacles(1,1),contexts.obstacles(2,1),100,'xb','linewidth',4);
        object2=scatter(contexts.obstacles(1,2),contexts.obstacles(2,2), 100, 'ob', 'linewidth',4);
        xlabel('Dimension 1', 'fontsize', fs);
        ylabel('Dimension 2', 'fontsize', fs);
        
        str = input('Do you want to continue? (y/n)', 's');
        if str ~= 'y'
            continue;
        end
    else
        contexts.obstacles = repmat([0.24 0.47, 0.64]',1,n_o);
        contexts.obstacles(3,:) = linspace(0, contexts.obstacles(3,1), n_o);
%         contexts.obstacles = rand(3,1);
    end
    
    %% Generate adapted trajectory
    [y_adapted, cost] = generateTraj(contexts, weights, config);
    
    %% Plot adapted trajectory
    if ~simulation_3D
        adapt=plot(y_adapted(:,1)', y_adapted(:,2)','linewidth',lw*2);
    else
        figure;
        plot3(refTraj(:,1)', refTraj(:,2)', refTraj(:,3)');
        hold on;
        plot3(y_adapted(:,1)', y_adapted(:,2)', y_adapted(:,3)');
        scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:), contexts.obstacles(3,:),100, 'xb', 'linewidth',4); 
        plot3(contexts.obstacles(1,:),contexts.obstacles(2,:), contexts.obstacles(3,:)); 
    end
    
    %% Learn the feedback rewards
    str = input('Do you want to improve the trajectory? (y/n)', 's');
    if str ~= 'y'
        lhndLegend=legend([imitation, adapt,object1,object2], 'Imitation Trajectory','Adaptation Trajectory','Obstacle 1','Obstacle 2');
        set(lhndLegend, 'fontsize', fs);
        set(gca, 'fontsize', fs);
        continue;
    end
    y_improved = getFeedback(y_adapted);
        
    d = 0;
    for t = 1:T
        d = d + sum((y_improved(t,:)-y_adapted(t,:)).^2);
    end
    err_dist(1,k) = d;
    
    if ~simulation_3D
    %% Plot improved trajectory
        improved=plot(y_improved(:,1)', y_improved(:,2)','--','linewidth',lw*2);
        lhndLegend=legend([imitation, adapt,improved, object1,object2], 'Imitation Trajectory','Adaptation Trajectory','Improved Trajectory','Obstacle 1','Obstacle 2');
        set(lhndLegend, 'fontsize', fs);
        set(gca, 'fontsize', fs);
  
    else 
        plot3(y_improved(:,1)', y_improved(:,2)',y_improved(:,3)');
    end
    
    %%% Learning iterations for single sample
    for j = 1:M  
        
        contexts.iteration = j;
        weights_improved = updateWeights(y_adapted, y_improved, contexts, weights, config);
        
        fprintf('Finished Learning Iteration: %d\n', j);
        weights = weights_improved
        
        %% Regenerate and plot adapted trajectory after learning
        [y_adapted, cost] = generateTraj(contexts, weights_improved, config);
        
        if ~simulation_3D
            figure;
            plot(refTraj(:,1)', refTraj(:,2)');
            hold on;
            plot(y_adapted(:,1)', y_adapted(:,2)');
            scatter(contexts.obstacles(1,:),contexts.obstacles(2,:), 100, 'xb', 'linewidth',4);
            plot(y_improved(:,1)', y_improved(:,2)');
        else
            figure;
            plot3(refTraj(:,1)', refTraj(:,2)',refTraj(:,3)');
            hold on;
            plot3(y_adapted(:,1)', y_adapted(:,2)', y_adapted(:,3)');
            scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:), contexts.obstacles(3,:), 100, 'xb', 'linewidth',4);
            plot3(y_improved(:,1)', y_improved(:,2)', y_improved(:,3)');
        end
        
        
        %% Learning error compared to improved trajectory
        d = 0;
        for t = 1:T
            d = d + sum((y_improved(t,:)-y_adapted(t,:)).^2);
        end
        err_dist(j+1,k) = d;
    end
    
    k = k+1;
    
    figure;
    plot(err_dist(:,i),'MarkerFaceColor','black','Marker','s','linewidth',lw*2);
    grid on;
    xlabel('Learning iterations', 'fontsize', fs);
    ylabel('Learning error', 'fontsize', fs);
    set(gca, 'fontsize', fs);
    
%     dist(i) = HausdorffDist(y_adapted, y_improved);
%     close all;    
end