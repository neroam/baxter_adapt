function hess = quadHess(U, lambda, contexts, weights, config)

%% Contexts extraction
x = contexts.xt;
y0 = contexts.yt;
Px = contexts.Px;
Pu = contexts.Pu;
dim = size(x,1);
h = contexts.horizon;
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;
s = config.scale;

%% Predict future outputs
Y = Px*x + Pu*U;

%%% Demonstration feature
A = cell(1, h);
for i = 1:h
%     A{i} = Q * diag(refVar((i-1)*dim+1:i*dim,1));
    A{i} = diag(weights(1:dim));
end
W = blkdiag(A{:});

hess = 2*Pu'*W*Pu;

%%% Obstacles feature
k = size(obstacles,2);
W = weights(dim+1:dim + k*dim,1);
for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);
    Put = Pu((i-1)*dim+1:i*dim,:);
    %%% k obstacles
    for j = 1:size(obstacles, 2)
        wj = W((j-1)*dim+1:j*dim,1)';
        obst = obstacles(:,j);
        hess = hess ...
            - (wj*Put)'*(...
                - 1/(norm(yt-obst)^3)*(yt-obst)'*Put*exp(-(yt-obst)'*(yt-obst)/d*s)...
                + 1/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(yt-obst)'*Put/d*s...
            )...
            + Put'*(yt-obst)/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)*wj*Put...
            + wj*(yt-obst)*Put'*(...
                + Put/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)...
                + (yt-obst)*(...
                    + (-3)/(norm(yt-obst)^5)*(yt-obst)'*Put*exp(-(yt-obst)'*(yt-obst)/d*s)...
                    + 1/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(yt-obst)'*Put/d*s...
                )...
            )...
            + (-2)*Put'*(yt-obst)/d*s*(...
                + exp(-(yt-obst)'*(yt-obst)/d*s)*(...
                    - wj*Put/norm(yt-obst)...
                    - wj*(yt-obst)/(norm(yt-obst)^3)*(yt-obst)'*Put...
                )...
                + (1-wj*(yt-obst)/norm(yt-obst))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(yt-obst)'*Put/d*s...
            )...
            +  (1-wj*(yt-obst)/norm(yt-obst))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(Put'*Put)/d*s;
        
    
    
%     
%             - (wj*Pu((i-1)*dim+1:i*dim,:))'/(norm(yt-obst)^2)*(yt-obst)'*Pu((i-1)*dim+1:i*dim,:)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/(norm(yt-obst)^3)*wj*Pu((i-1)*dim+1:i*dim,:)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             - (wj*Pu((i-1)*dim+1:i*dim,:))'/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(yt-obst)'*Pu((i-1)*dim+1:i*dim,:)...
%             + wj*(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*Pu((i-1)*dim+1:i*dim,:)/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + wj*(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)*(-3)/(norm(yt-obst)^4)*(yt-obst)'*Pu((i-1)*dim+1:i*dim,:)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + wj*(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*(yt-obst)'*Pu((i-1)*dim+1:i*dim,:)...
%             + (-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s*(...
%             - (wj*Pu((i-1)*dim+1:i*dim,:))'/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + wj*(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             - wj*(yt-obst)/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s ...
%             )' ...
%             - wj*(yt-obst)/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*Pu((i-1)*dim+1:i*dim,:)/d*s;
    end
end

%%% Smooth feature
W = weights((k+1)*dim+1,1);
hess = hess + W*2*Pu(1:dim,:)'*Pu(1:dim,:);
for i = 2:h
    t = (i-1)*dim+1:i*dim;
    hess = hess + W*2*(Pu(t,:)-Pu(t-dim,:))'*(Pu(t,:)-Pu(t-dim,:));
end

%%% Control smooth feature
% A = cell(1, h);
% for i = 1:h
%     A{i} = diag(weights((k+1)*dim+1:(k+1)*dim+dim,1));
% end
% W = blkdiag(A{:});
% hess = hess + 2*W;

%%% Inequality constraints
for i = 1:h
    hess = hess + lambda.ineqnonlin(i)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*Pu((i-1)*dim+1:i*dim,:);  
end

%% Normalize by steps
hess = hess / h;



end