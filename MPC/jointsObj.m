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
[h, dim] = size(contexts.refTraj);
joints_dim = size(Px,2);
refTraj = reshape(contexts.refTraj',[],1);
refVar = reshape(contexts.refVar',[],1);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;
s = config.scale;

%% Predict future outputs
Y_joints = Px*x + Pu*U;

Y = zeros(dim*h, 1);
gradY = zeros(dim*h, joints_dim*h);

for i = 1:h
    t = (i-1)*dim+1:i*dim;
    [pose, ~, gradPose] = forwardKine(Y_joints((i-1)*joints_dim+1:i*joints_dim,1));
    Y(t,:) = pose(1:3,:);
    gradY(t,(i-1)*joints_dim+1:i*joints_dim) = squeeze(gradPose(1:3,4,:));
end

%%% Demonstration feature
A = cell(1, h);
for i = 1:h
    A{i} = diag(weights(1:dim)).*diag(refVar((i-1)*dim+1:i*dim,1));
%     A{i} = diag(weights(1:dim));
end
W = blkdiag(A{:});

f = (Y-refTraj)'*W*(Y-refTraj);
g = 2*Pu'*gradY'*W*(Y-refTraj);

%%% Smooth feature
W = weights(dim+1,1);

% [y0, ~,~] = forwardKine(y0_joints);
% y0 = y0(1:3,:);
% f = f + W*(Y(1:dim,1)-y0)'*(Y(1:dim,1)-y0);
% g = g + W*2*Pu(1:joints_dim,:)'*gradY(1:dim, 1:joints_dim)'*(Y(1:dim,1) - y0);
% 
% for i = 2:h
%     t = (i-1)*dim+1:i*dim;
%     joints_t = (i-1)*joints_dim+1:i*joints_dim;
%     f = f + W*(Y(t,1) - Y(t-dim,1))'*(Y(t,1) - Y(t-dim,1));
%     g = g + W*2*(Pu(joints_t,:)'*gradY(t,joints_t)'-Pu(joints_t-joints_dim,:)'*gradY(t-dim,joints_t-joints_dim)')*(Y(t,1) - Y(t-dim,1));
% end

f = f + W*(Y_joints(1:joints_dim,1)-y0_joints)'*(Y_joints(1:joints_dim,1)-y0_joints);
g = g + W*2*Pu(1:joints_dim,:)'*(Y_joints(1:joints_dim,1) - y0_joints);

for i = 2:h
    t = (i-1)*joints_dim+1:i*joints_dim;
    f = f + W*(Y_joints(t,1) - Y_joints(t-joints_dim,1))'*(Y_joints(t,1) - Y_joints(t-joints_dim,1));
    g = g + W*2*(Pu(t,:)-Pu(t-joints_dim,:))'*(Y_joints(t,1) - Y_joints(t-joints_dim,1));
end

%%% Obstacles feature
k = size(obstacles,2);
W = weights(dim+2:dim +1+k*(dim+2),1);
for i = 1:h
    t = (i-1)*dim+1:i*dim;
    joints_t = (i-1)*joints_dim+1:i*joints_dim;
    yt = Y(t,1);
    yt_ref = refTraj((i-1)*dim+1:i*dim,1);
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
% 
%%% Boarder feature
offset = dim +1+k*(dim+2);
d = config.dboarder;
W = weights(offset + 1,1);
boarder = contexts.boarder;
for i = 1:h
    t = (i-1)*dim+1;
    joints_t = (i-1)*joints_dim+1;
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
    joints_t = i*joints_dim;
    yt = Y(t,1);
    f = f + W*((yt-contexts.ground).^2)/h;
    g = g + W*2*Pu(joints_t,:)'*gradY(t,joints_t)'*(yt-contexts.ground)/h;
end

end