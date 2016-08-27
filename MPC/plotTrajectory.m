function plotTrajectory(y_imit,y_adapt,contexts,config, y_improved)

lw = 2;
fs = 26;

%% Plot adapted trajectory
figure;
imitation=plot3(y_imit(:,1)', y_imit(:,2)', y_imit(:,3)' ,'-.','linewidth',lw*2);
hold on;
objects=scatter3(contexts.obstacles(1,:),contexts.obstacles(2,:),contexts.obstacles(3,:),100,'xb','linewidth',4);
% object2=scatter3(contexts.obstacles(1,2),contexts.obstacles(2,2),contexts.obstacles(3,1), 100, 'ob', 'linewidth',4);
xlabel('X (m)', 'fontsize', fs);
ylabel('Y (m)', 'fontsize', fs);
zlabel('Z (m)', 'fontsize', fs);

r = sqrt(config.dmin) - 0.1;
[x,y,z] = sphere(30);
x = x*r+contexts.obstacles(1);
y = y*r+contexts.obstacles(2);
z = z*r+contexts.obstacles(3);
surface(x,y,z,'FaceColor','none','EdgeColor',[0.8,0.8,0.8]);

if nargin < 5
    adapt=plot3(y_adapt(:,1)', y_adapt(:,2)', y_adapt(:,3)','linewidth',lw*2);
    lhndLegend=legend([imitation, adapt,objects], 'Imitation Trajectory','Adaptation Trajectory','Obstacles');
else
   adapt=plot3(y_adapt(:,1)', y_adapt(:,2)', y_adapt(:,3)','linewidth',lw*2);
   improved = plot3(y_improved(:,1)', y_improved(:,2)', y_improved(:,3)','--','linewidth',lw*2);
   lhndLegend=legend([imitation, adapt,improved, objects], 'Imitation Trajectory','Adaptation Trajectory','Feedback Trajectory','Obstacles');
end

set(lhndLegend, 'fontsize', fs);
set(gca, 'fontsize', fs);

