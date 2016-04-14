function plotTrajectory(y_imit,y_adapt,contexts)

lw = 2;
fs = 26;

%% Plot adapted trajectory
figure;
imitation=plot3(y_imit(:,1)', y_imit(:,2)', y_imit(:,3)' ,'--','linewidth',lw*2);
hold on;
adapt=plot3(y_adapt(:,1)', y_adapt(:,2)', y_adapt(:,3)','linewidth',lw*2);
objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
% object2=scatter3(contexts.obstacles(1,2),contexts.obstacles(2,2),contexts.obstacles(3,1), 100, 'ob', 'linewidth',4);
lhndLegend=legend([imitation, adapt,objects], 'Imitation Trajectory','Adaptation Trajectory','Obstacles');
set(lhndLegend, 'fontsize', fs);
set(gca, 'fontsize', fs);