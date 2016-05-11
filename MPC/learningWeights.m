function [w_improved] = learningWeights(y, y_improved, contexts, weights, config)

%%% Contexts extraction
y_ref = contexts.refTraj;
[T, joints_dim] = size(y_ref); 
h = T - 1;
r = config.alpha;
phi = 1/sqrt(contexts.iteration);
% phi = 1;

w_improved = weights;

%% Update weights for demo feature
feature_improved = 0;
feature_orig = 0;

y_refVar = contexts.refVar;
for i = 1:joints_dim
    y_refVar(:,i) =  exp(-contexts.refVar(:,i));
end

for t = 2:1+h
%     feature_improved = feature_improved - ((y_improved(t,:)-y_ref(t,:)).^2)';
%     feature_orig = feature_orig - ((y(t,:)-y_ref(t,:)).^2)';
    
    feature_improved = feature_improved - ((y_improved(t,:)-y_ref(t,:)).^2)'.*y_refVar(t,:)';
    feature_orig = feature_orig - ((y(t,:)-y_ref(t,:)).^2)'.*y_refVar(t,:)';
end

w_improved(1:joints_dim) = weights(1:joints_dim,1) + r*phi*(feature_improved - feature_orig)/h;

%% Update weights for smooth feature
feature_improved = 0;
feature_orig = 0;
for i = 2:1+h
    feature_improved = feature_improved - (y_improved(i,1)' - y_improved(i-1,1)')'*(y_improved(i,1)' - y_improved(i-1,1)');
    feature_orig = feature_orig - (y(i,1)'-y(i-1,1)')'*(y(i,1)'-y(i-1,1)');
end
w_improved(joints_dim+1,1) = weights(joints_dim+1,1) + r*phi*(feature_improved - feature_orig);

%% Update weights for obstacle feature
d = config.dmin;
s = config.scale;
obstacles = contexts.obstacles;

dim = 3;
offset = joints_dim+1;
k = size(obstacles,2);
feature_improved = zeros(k*(dim+2),1);
feature_orig = zeros(k*(dim+2),1);

Y = zeros(h, dim);
Y_improved = zeros(h,dim);

for i = 1:T
    [pose,~] = forwardKine(y(i,:)');
    Y(i,:) = pose(1:3,1)';
    [pose, ~] = forwardKine(y_improved(i,:)');
    Y_improved(i,:) = pose(1:3,1)';
end

for t = 2:1+h
    yt_improved = Y_improved(t,:)';
    yt = Y(t,:)';
    yt_ref = forwardKine(y_ref(t,:)');
    yt_ref = yt_ref(1:3,1);

    %%% k obstacles
    for j = 1:size(obstacles, 2)
        wj = (j-1)*(dim+2)+1:j*(dim+2);
        obst = obstacles(:,j);
        
        feature_improved(wj,1) = feature_improved(wj,1) + [-sqrt(d)*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s); norm(yt_improved-obst)*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s);-(yt_ref-yt_improved)*exp(-(yt-obst)'*(yt-obst)/d*s)];
        feature_orig(wj,1) = feature_orig(wj,1) + [-sqrt(d)*exp(-(yt-obst)'*(yt-obst)/d*s); norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s);-(yt_ref-yt)*exp(-(yt-obst)'*(yt-obst)/d*s)];
        
    end
end
w_improved(offset+1:offset+k*(dim+2),1) = weights(offset+1:offset+k*(dim+2),1) + r*phi*(feature_improved - feature_orig);


%% Update weights for boarder feature
offset = offset+k*(dim+2);
d = config.dboarder;
boarder = contexts.boarder;
feature_improved = 0;
feature_orig = 0;
for t = 1:h
    yt = Y(t,:)';
    yt_improved = Y_improved(t,:)';
    feature_improved = feature_improved - (exp(-(yt_improved(2)-boarder(1))^2/d) + exp(-(yt_improved(2)-boarder(2))^2/d));
    feature_orig = feature_orig - (exp(-(yt(2)-boarder(1))^2/d) + exp(-(yt(2)-boarder(2))^2/d));
end

w_improved(offset+1,1) = weights(offset+1,1) + r*phi*(feature_improved - feature_orig);

%% Update weights for boarder safety feature
offset = offset +1;
feature_change = 0;
for t = 1:h
    yt = Y(t,:)';
    yt_improved = Y_improved(t,:)';
    feature_change = feature_change + (-((yt_improved(3)-contexts.ground).^2) + ((yt(3)-contexts.ground).^2));
end
w_improved(offset+1,1) = w_improved(offset+1,1) + feature_change/h;



%% Project new weights to convex set.
%% Using fmincon
options = optimoptions(@fmincon,'Algorithm','interior-point',...
    'GradObj','on','GradConstr','on');
fun = @(U)projectObj(U,w_improved);
nonlconstr = @(U)projectConstr(U,contexts);
U0 = w_improved

[U,J,eflag,output,lambda] = fmincon(fun,U0,...
    [],[],[],[],[],[],nonlconstr,options);

w_improved = U

end