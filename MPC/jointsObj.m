function [f, g] = jointsObj(U, contexts, weights, config)

%U (joints_dim*h, 1)

% contexts:
%   Px (joints_dim*h, joints_dim)
%   Pu (joints_dim*h, joints_dim*h)
%   xt  (joints_dim, 1)
%   refTraj  (h, dim)
%   refVar  (h, dim)
%   obstacles (dim, k)

% config:
%   dmin (min distance to avoid obstacle)
%   scale (scale for features decaying)

%% Contexts extraction
x = contexts.xt;
y0_joints = contexts.yt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, joints_dim] = size(contexts.refTraj);
refTraj = reshape(contexts.refTraj',[],1);
refVar = reshape(contexts.refVar',[],1);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;
s = config.scale;

%% Predict future outputs
Y_joints = Px*x + Pu*U;

%%% Demonstration feature
A = cell(1, h);
for i = 1:h
    A{i} = diag(weights(1:joints_dim)).*diag(refVar((i-1)*joints_dim+1:i*joints_dim,1));
%     A{i} = diag(weights(1:dim));
end
W = blkdiag(A{:});

f = (Y_joints-refTraj)'*W*(Y_joints-refTraj)/h;
g = 2*Pu'*W*(Y_joints-refTraj)/h;

%%% Smooth feature
W = weights(joints_dim+1,1);

f = f + W*(Y_joints(1:joints_dim,1)-y0_joints)'*(Y_joints(1:joints_dim,1)-y0_joints);
g = g + W*2*Pu(1:joints_dim,:)'*(Y_joints(1:joints_dim,1) - y0_joints);

for i = 2:h
    t = (i-1)*joints_dim+1:i*joints_dim;
    f = f + W*(Y_joints(t,1) - Y_joints(t-joints_dim,1))'*(Y_joints(t,1) - Y_joints(t-joints_dim,1));
    g = g + W*2*(Pu(t,:)-Pu(t-joints_dim,:))'*(Y_joints(t,1) - Y_joints(t-joints_dim,1));
end

%%% Obstacles feature
dim = 3;
k = size(obstacles,2);
W = weights(joints_dim+2:joints_dim+1+k*(dim+2),1);

Y = zeros(dim*h, 1);
gradY = zeros(dim*h, joints_dim*h);

for i = 1:h
    t = (i-1)*dim+1:i*dim;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    [pose,gradPose] = forwardKine(Y_joints(joints_t,1));
    Y(t,:) = pose(1:3,1);
    gradY(t,joints_t) = gradPose(1:3,:);
end

for i = 1:h
    t = (i-1)*dim+1:i*dim;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    yt = Y(t,1);
    yt_ref = forwardKine(refTraj(joints_t,1));
    yt_ref = yt_ref(1:3,1);
    %%% k obstacles
    for j = 1:size(obstacles, 2)
        wk1 = W((j-1)*(dim+2)+1,1);
        wk2 = W((j-1)*(dim+2)+2,1);
        wj = W((j-1)*(dim+2)+3:j*(dim+2),1)';

        obst = obstacles(:,j);

        f = f + (wk1*sqrt(d)-wk2*norm(yt-obst)-wj*(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s);
        g = g + (wk2/norm(yt-obst)*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt-obst)-Pu(joints_t,:)'*gradY(t,joints_t)'*wj')*exp(-(yt-obst)'*(yt-obst)/d*s)...
            + (wk1*sqrt(d)-wk2*norm(yt-obst)-wj*(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt-obst)/d*s; 
        
    end
    
end

%%% Boarder feature
offset = joints_dim+1+k*(dim+2);
d = config.dboarder;
W = weights(offset + 1,1);
boarder = contexts.boarder;
for i = 1:h
    t = (i-1)*dim+2;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    yt = Y(t,1);
    f = f + W*(exp(-(yt(1)-boarder(1))^2/d) + exp(-(yt(1)-boarder(2))^2/d));
    g = g + W*(exp(-(yt(1)-boarder(1))^2/d)*(-2)/d*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt(1)-boarder(1))...
        + exp(-(yt(1)-boarder(2))^2/d)*(-2)/d*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt(1)-boarder(2)));
end

%%% Safety board feature
offset = offset + 1;
W = weights(offset + 1,1);
for i = 1:h
    t = i*dim;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    yt = Y(t,1);
    f = f + W*((yt-contexts.ground).^2)/h;
    g = g + W*2*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt-contexts.ground)/h;
end

end