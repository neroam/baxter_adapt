function [f, feq, gradf, gradfeq] = jointsConstr(U, contexts, config)
% Constraints of obstacles

%% Contexts extraction
x = contexts.xt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, joints_dim] = size(contexts.refTraj);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;

%% Predict future outputs
Y_joints = Px*x + Pu*U;

pose_dim = 7;
Y = zeros(pose_dim*h, 1);
gradY = zeros(pose_dim*h, joints_dim*h);

for i = 1:h
    t = (i-1)*pose_dim+1:pose_dim*i;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    [pose, gradPose] = forwardKine(Y_joints(joints_t,1));
    Y(t,:) = pose;
    gradY(t,joints_t) = gradPose;
    
end

k = size(obstacles,2);

f = zeros(1,h*k+h);
gradf = zeros(length(U),h*k+h);

%% Avoid obstacles constraints
for j = 1:k
    obst = obstacles(:,j);
    for i = 1:h
        t = (i-1)*pose_dim+1:(i-1)*pose_dim+3;
        joints_t = (i-1)*joints_dim+1:i*joints_dim;
        yt = Y(t,1);
        f((j-1)*h+i) = d - (yt-obst)'*(yt-obst);
        gradf(:,(j-1)*h+i) = -2*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt-obst);
    end
end

%% Above the table ground
for i = 1:h
    t = (i-1)*pose_dim+3;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    yt = Y(t,1);
    f(h*k+i) = contexts.ground - yt;
    gradf(:,h*k+i) = -Pu(joints_t,:)'*gradY(t,joints_t)';
end


feq = [];
gradfeq = [];

%% Terminal state constraint
if contexts.terminal
    
    f(h*k+h) = 0;
    gradf(:,h*k+h) = 0;
    
    joints_t = (h-1)*joints_dim+1:h*joints_dim;
    yh = Y_joints(joints_t,1);
    feq = [feq (yh-contexts.refTraj')'*(yh-contexts.refTraj')];
    gradfeq = [gradfeq 2*Pu(joints_t,:)'*(yh-contexts.refTraj')];

end

end