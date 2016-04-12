function [f, feq, gradf, gradfeq] = jointsConstr(U, contexts, config)
% Constraints of obstacles

%% Contexts extraction
x = contexts.xt;
y0_joints = contexts.yt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, dim] = size(contexts.refTraj);
joints_dim = size(Px,2);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;

%% Predict future outputs
Y_joints = Px*x + Pu*U;

Y = zeros(dim*h, 1);
gradY = zeros(dim*h, joints_dim*h);
rotationY = zeros(9*h,1);
gradRotY = zeros(9*h, joints_dim*h);

for i = 1:h
    t = (i-1)*dim+1:i*dim;
    [pose, rotMat, gradPose] = forwardKine(Y_joints((i-1)*joints_dim+1:i*joints_dim,1));
    Y(t,:) = pose(1:3,:);
    gradY(t,(i-1)*joints_dim+1:i*joints_dim) = squeeze(gradPose(1:3,4,:));
    rotationY((i-1)*9+1:i*9,:) = rotMat(:);
    gradRotY((i-1)*9+1:i*9,(i-1)*joints_dim+1:i*joints_dim) = reshape(gradPose(1:3,1:3,:),9,7);
    
end

k = size(obstacles,2);

f = zeros(1,h*k);
gradf = zeros(length(U),h*k);

%% Avoid obstacles constraints
for j = 1:k
    obst = obstacles(:,j);
    for i = 1:h
        yt = Y((i-1)*dim+1:i*dim,1);
        f((j-1)*h+i) = d - (yt-obst)'*(yt-obst);
        gradf(:,(j-1)*h+i) = -2*Pu((i-1)*joints_dim+1:i*joints_dim,:)'*gradY((i-1)*dim+1:i*dim,(i-1)*joints_dim+1:i*joints_dim)'*(yt-obst);
    end
end

feq = [];
gradfeq = [];

feq = zeros(1,h);
gradfeq = zeros(length(U),h);

%% Rotation of end_effector constraints
[~, Y0Mat, ~] = forwardKine(y0_joints);
rotY0 = Y0Mat(:);

for i = 1:h
    rotYi = rotationY((i-1)*9+1:i*9,1);
    feq(i) = (rotYi-rotY0)'*(rotYi-rotY0);
    gradfeq(:,i) = 2*Pu((i-1)*joints_dim+1:i*joints_dim,:)'*gradRotY((i-1)*9+1:i*9,(i-1)*joints_dim+1:i*joints_dim)'*(rotYi-rotY0);
end

%% Terminal state constraint
if contexts.terminal
    yh = Y((h-1)*dim+1:h*dim,1);
    feq = [feq (yh-contexts.refTraj(end,:)')'*(yh-contexts.refTraj(end,:)')];
    gradfeq = [gradfeq 2*Pu((h-1)*joints_dim+1:h*joints_dim,:)'*gradY((h-1)*dim+1:h*dim,(h-1)*joints_dim+1:h*joints_dim)'*(yh-contexts.refTraj(end,:)')];

end

end