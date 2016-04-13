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

f = zeros(1,h*k);
gradf = zeros(length(U),h*k);

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

feq = [];
gradfeq = [];

% feq = zeros(1,h);
% gradfeq = zeros(length(U),h);
% 
% %% Rotation of end_effector constraints
% [y0_pose,~] = forwardKine(contexts.start_joints);
% rotY0 = y0_pose(4:7,1);
% 
% for i = 1:h
%     rotYi = rotY((i-1)*4+1:i*4,1);
%     feq(i) = (rotYi-rotY0)'*(rotYi-rotY0);
%     gradfeq(:,i) = 2*Pu((i-1)*joints_dim+1:i*joints_dim,:)'*gradRotY((i-1)*4+1:i*4,(i-1)*joints_dim+1:i*joints_dim)'*(rotYi-rotY0);
% end

%% Terminal state constraint
if contexts.terminal
    joints_t = (h-1)*joints_dim+1:h*joints_dim;
    yh = Y_joints(joints_t,1);
    feq = [feq (yh-contexts.refTraj')'*(yh-contexts.refTraj')];
    gradfeq = [gradfeq 2*Pu(joints_t,:)'*(yh-contexts.refTraj')];

end

end