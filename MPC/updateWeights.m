function [w_improved] = updateWeights(y, y_improved, contexts, weights, config)

%% Score function as linear of weights and feature kernel. score function = - cost function
%%% Contexts extraction
y_ref = contexts.refTraj;
[T, dim] = size(y_ref); 
w_improved = weights;
h = T - 1;
r = config.alpha;
% phi = 1/sqrt(contexts.iteration);
phi = 1;


%% Update weights for demo feature
feature_improved = 0;
feature_orig = 0;
for t = 2:1+h
    feature_improved = feature_improved + ((y_improved(t,:)-y_ref(t,:)).^2)'.*contexts.refVar(t,:)';
    feature_orig = feature_orig + ((y(t,:)-y_ref(t,:)).^2)'.*contexts.refVar(t,:)';
end

w_improved(1:dim) = weights(1:dim,1) + r*phi*(feature_improved - feature_orig);

% feature_improved = -sum((y_improved(2:1+h,:)-y_ref(2:1+h,:)).^2,1)';
% feature_orig = -sum((y(2:1+h,:)-y_ref(2:1+h,:)).^2,1)';
% w_improved(1:dim) = weights(1:dim,1) + phi*(feature_improved - feature_orig);
% w_improved(1:dim) = weights(1:dim) + r*phi*repmat(sum(feature_improved - feature_orig),2,1);

%% Update weights for smooth feature
feature_improved = 0;
feature_orig = 0;
for i = 2:1+h
    feature_improved = feature_improved - (y_improved(i,1)' - y_improved(i-1,1)')'*(y_improved(i,1)' - y_improved(i-1,1)');
    feature_orig = feature_orig - (y(i,1)'-y(i-1,1)')'*(y(i,1)'-y(i-1,1)');
end
w_improved(dim+1,1) = weights(dim+1,1) + r*phi*(feature_improved - feature_orig);

%% Update weights for obstacle feature
d = config.dmin;
s = config.scale;
obstacles = contexts.obstacles;

k = size(obstacles,2);
feature_improved = zeros(k*(dim+2),1);
feature_orig = zeros(k*(dim+2),1);
offset = dim+1;
for t = 2:1+h
    yt_improved = y_improved(t,:)';
    yt = y(t,:)';
    yt_ref = y_ref(t,:)';

    %%% k obstacles
    for j = 1:size(obstacles, 2)
        wj = (j-1)*(dim+2)+1:j*(dim+2);
        obst = obstacles(:,j);
%         feature_improved(wj,1) = feature_improved(wj,1) + [-1; (yt_improved-obst)]*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s);
%         feature_orig(wj,1) = feature_orig(wj,1) + [-1; (yt-obst)]*exp(-(yt-obst)'*(yt-obst)/d*s);
        
        feature_improved(wj,1) = feature_improved(wj,1) + [-sqrt(d)*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s); norm(yt_improved-obst)*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s);-(yt_ref-yt_improved)*exp(-(yt-obst)'*(yt-obst)/d*s)];
        feature_orig(wj,1) = feature_orig(wj,1) + [-sqrt(d)*exp(-(yt-obst)'*(yt-obst)/d*s); norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s);-(yt_ref-yt)*exp(-(yt-obst)'*(yt-obst)/d*s)];
        
%         feature_improved(wk,1) = feature_improved(wk,1) + (yt_improved-obst)/norm(yt_improved-obst)*exp(-(yt_improved-obst)'*(yt_improved-obst)/d*s);
%         feature_orig(wk,1) = feature_orig(wk,1) + (yt-obst)/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s);
    end
end
w_improved(offset+1:offset+k*(dim+2),1) = weights(offset+1:offset+k*(dim+2),1) + r*phi*(feature_improved - feature_orig);

% /norm((feature_improved - feature_orig))

%% Update weights for boarder feature
offset = offset+k*(dim+2);
d = config.dboarder;
boarder = contexts.boarder;
feature_improved = 0;
feature_orig = 0;
for t = 1:h
    yt = y(t,:)';
    yt_improved = y_improved(t,:)';
    feature_improved = feature_improved - (exp(-(yt_improved(1)-boarder(1))^2/d) + exp(-(yt_improved(1)-boarder(2))^2/d));
    feature_orig = feature_orig - (exp(-(yt(1)-boarder(1))^2/d) + exp(-(yt(1)-boarder(2))^2/d));
end

w_improved(offset+1,1) = weights(offset+1,1) + r*phi*(feature_improved - feature_orig);

%% Update weights for boarder safety feature
offset = offset +1;
feature_change = 0;
for t = 1:h
    yt = y(t,:)';
    yt_improved = y_improved(t,:)';
    feature_change = feature_change + (-((yt_improved(2)-contexts.ground).^2) + ((yt(2)-contexts.ground).^2))/h;
end
w_improved(offset+1,1) = w_improved(offset+1,1) + feature_change;



%% Project new weights to convex set.
%%% obstacle weights are unit vector. others should be >= 0.

%% Using fmincon
options = optimoptions(@fmincon,'Algorithm','interior-point',...
    'GradObj','on','GradConstr','on');
fun = @(U)projectObj(U,w_improved);
nonlconstr = @(U)projectConstr(U,contexts);
U0 = w_improved

[U,J,eflag,output,lambda] = fmincon(fun,U0,...
    [],[],[],[],[],[],nonlconstr,options);

w_improved = U

    
% %% Update weights for controls feature
% A = diag(0.9*ones(dim,1));
% B = diag(ones(dim,1));
% C = diag(ones(dim,1));
% 
% Px = C*A;
% Pu = C*B;
% 
% P = C;
% for k=1:h;
%     Puterm = P*B;
%     for j=k:h;
%         vrow=(j-1)*dim+1:j*dim;
%         vcol=(j-k)*dim+1:(j-k+1)*dim;
%         Pu(vrow,vcol)=Puterm;
%     end
%     P=P*A;
%     vrow=(k-1)*dim+1:k*dim;
%     Px(vrow,1:dim) = P;
% end
%     
% x0 = (inv(C)*y(1,:)');
% U = reshape(inv(Pu)*(reshape(y(2:1+h,:)',[],1)-Px*x0),dim,[])';
% U_improved = reshape(inv(Pu)*(reshape(y_improved(2:1+h,:)',[],1)-Px*x0),dim,[])';
% 
% feature_improved_control = -sum(U_improved.^2,1);
% feature_control = -sum(U.^2,1);
% % w_improved(2*dim+1:3*dim) = w(2*dim+1:3*dim) + feature_improved_control/h - feature_control/h;
% w_improved(2*dim+1:3*dim) = w(2*dim+1:3*dim) + repmat(sum(feature_improved_control/h - feature_control/h),1,2);





% %% Update weights for obstacle feature in gradient direction
% w_obstacle = w(dim+1:2*dim);
% obstacles = repmat(obstacles', T, 1);
% distance_improved = (y_improved-obstacles).^2;
% distance_orig = (y-obstacles).^2;
% g = exp(-[0 0]*distance_improved')*distance_improved - exp(-[0 0]*distance_orig')*distance_orig
% w_improved(dim+1:2*dim) = w(dim+1:2*dim) + g;

%% Update weights for obstacle feature
%%% Extend weights according to approximation of exponential

% w_obstacle = w(dim+1:2*dim);
% for i = 1:dim
%     for j = i:dim
%         w_extend(1, (i-1)*dim+j) = w_obstacle(i)*w_obstacle(j);
%     end
% end
% 
% %%% Update extended weights as they are linear
% feature_obstacle_improved = zeros(T,dim);
% feature_obstacle_orig = zeros(T,dim);
% 
% for t = 1:T
%     yt = y_improved(t,:)';
%     if (1 - (yt-obstacles)'*diag(w_obstacle)*(yt-obstacles)/2) > 0
%         feature_obstacle_improved(t,:) = ((yt-obstacles).^2)';
%     end
%     yt = y(t,:)';
%     if (1 - (yt-obstacles)'*diag(w_obstacle)*(yt-obstacles)/2) > 0
%         feature_obstacle_orig(t,:) = ((yt-obstacles).^2)';
%     end
% end
% %%% only considering points that are within range of obstacles and affecting reference trajectory
% 
% 
% for i = 1:dim
%     for j = i:dim
%         feature_improved_obstacle(1, (i-1)*dim+j) = -sum(feature_obstacle_improved(:,i).*feature_obstacle_improved(:,j)/4,1);
%         feature_orig_obstacle(1, (i-1)*dim+j) = -sum(feature_obstacle_orig(:,i).*feature_obstacle_orig(:,j)/4,1);
%         if i ~= j
%             feature_improved_obstacle(1, (i-1)*dim+j) = 2*feature_improved_obstacle(1, (i-1)*dim+j);
%             feature_orig_obstacle(1, (i-1)*dim+j) = 2*feature_orig_obstacle(1, (i-1)*dim+j);
%         end
%     end
% end
% 
% w_obstacle
% w_obstacle_improved = w_obstacle + sum(feature_obstacle_improved,1) - sum(feature_obstacle_orig,1)
% w_extend
% w_extend_improved = w_extend + feature_improved_obstacle - feature_orig_obstacle
% 
% % %%% Estimate the weights back
% % for i = 1:dim
% %     for j = i:dim
% %         if i == j
% %             w_obstacle_improved(1,i) = w_obstacle_improved(1, i) + sqrt(max(w_extend_improved(1, (i-1)*dim+j),0));
% %         else
% %             ratio = w_obstacle(i) / w_obstacle(j);
% %             wj = sqrt(max(w_extend_improved(1, (i-1)*dim+j)/ratio,0));
% %             w_obstacle_improved(1,j) = w_obstacle_improved(1,j) + wj;
% %             w_obstacle_improved(1,i) = w_obstacle_improved(1,i) + max(w_extend_improved(1,(i-1)*dim+j)/wj, 0);
% %         end
% %     end
% % end
% % 
% % w_obstacle_improved = w_obstacle_improved/(dim+1);
% 
% w_improved(dim+1:2*dim) = w_obstacle_improved;

end