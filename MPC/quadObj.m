function [f, g] = quadObj(U, contexts, weights, config)

%U (dim*h, 1)

% contexts:
%   Px (dim*h, dim)
%   Pu (dim*h, dim*h)
%   xt  (dim, 1)
%   refTraj  (h, dim)
%   refVar  (h, dim)
%   obstacles (dim, k)

% config:
%   dmin (min distance to avoid obstacle)
%   scale (scale for features decaying)

%% Contexts extraction
x = contexts.xt;
y0 = contexts.yt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, dim] = size(contexts.refTraj);
refTraj = reshape(contexts.refTraj',[],1);
refVar = reshape(contexts.refVar',[],1);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;
s = config.scale;

%% Predict future outputs
Y = Px*x + Pu*U;

%%% Demonstration feature
A = cell(1, h);
for i = 1:h
    A{i} = diag(weights(1:dim)).*diag(refVar((i-1)*dim+1:i*dim,1));
%     A{i} = diag(weights(1:dim));
end
W = blkdiag(A{:});

f = (Y-refTraj)'*W*(Y-refTraj);
g = 2*Pu'*W*(Y-refTraj);

%%% Smooth feature
W = weights(dim+1,1);
f = f + W*(Y(1:dim,1)-y0)'*(Y(1:dim,1)-y0);
g = g + W*2*Pu(1:dim,:)'*(Y(1:dim,1) - y0);

for i = 2:h
    t = (i-1)*dim+1:i*dim;
    f = f + W*(Y(t,1) - Y(t-dim,1))'*(Y(t,1) - Y(t-dim,1));
    g = g + W*2*(Pu(t,:)-Pu(t-dim,:))'*(Y(t,1) - Y(t-dim,1));
end

%%% Obstacles feature
k = size(obstacles,2);
W = weights(dim+2:dim +1+k*(dim+2),1);
for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);
    yt_ref = refTraj((i-1)*dim+1:i*dim,1);
    %%% k obstacles
    for j = 1:size(obstacles, 2)
        wk1 = W((j-1)*(dim+2)+1,1);
        wk2 = W((j-1)*(dim+2)+2,1);
        wj = W((j-1)*(dim+2)+3:j*(dim+2),1)';

        obst = obstacles(:,j);
%         f = f + (-wj*(yt-yt_ref)/norm(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s);
%         g = g -(wj*Pu((i-1)*dim+1:i*dim,:))'/norm(yt-yt_ref)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + wj*(yt-yt_ref)*Pu((i-1)*dim+1:i*dim,:)'*(yt-yt_ref)/(norm(yt-yt_ref)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + (-wj*(yt-yt_ref)/norm(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s;
        
%         f = f + (wk*norm(yt-obst)-wj*(yt-obst))*exp(-(yt-obst)'*(yt-obst)/d*s);
%         g = g + (1/norm(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)-(wj*Pu((i-1)*dim+1:i*dim,:))')*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + (wk*norm(yt-obst)-wj*(yt-obst))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s;
        
%         f = f + (wk-wj*(yt-yt_ref))^2*exp(-(yt-obst)'*(yt-obst)/d*s);
%         g = g + 2*(wk-wj*(yt-yt_ref))*(-wj*Pu((i-1)*dim+1:i*dim,:))'*exp(-(yt-obst)'*(yt-obst)/d*s)...
%             + (wk-wj*(yt_ref-yt))^2*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s;
        
        f = f + (wk1*sqrt(d)-wk2*norm(yt-obst)-wj*(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s);
        g = g + (wk2/norm(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)-(wj*Pu((i-1)*dim+1:i*dim,:))')*exp(-(yt-obst)'*(yt-obst)/d*s)...
            + (wk1*sqrt(d)-wk2*norm(yt-obst)-wj*(yt-yt_ref))*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s;
% 
%         if 1-(yt-obst)'*(yt-obst)/(d*s) > 0
%             f = f + (wj*(yt_ref-yt))*();
%             g = g + (-(wj*Pu((i-1)*dim+1:i*dim,:))');
%         end
        
        
    end
    
end

%%% Boarder feature
offset = dim +1+k*(dim+2);
d = config.dboarder;
W = weights(offset + 1,1);
boarder = contexts.boarder;
for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);
    f = f + W*(exp(-(yt(1)-boarder(1))^2/d) + exp(-(yt(1)-boarder(2))^2/d));
    g = g + W*(exp(-(yt(1)-boarder(1))^2/d)*(-2)/d*Pu((i-1)*dim+1,:)'*(yt(1)-boarder(1))...
        + exp(-(yt(1)-boarder(2))^2/d)*(-2)/d*Pu((i-1)*dim+1,:)'*(yt(1)-boarder(2)));
end

%%% Safety board feature
offset = offset + 1;
W = weights(offset + 1,1);
for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);
    f = f + W*((yt(2)-contexts.ground).^2)/h;
    g = g + W*2*Pu((i-1)*dim+2,:)'*(yt(2)-contexts.ground)/h;
end

%%% Control smooth feature
% A = cell(1, h);
% for i = 1:h
%     A{i} = diag(weights((k+1)*dim+1:(k+1)*dim+dim,1));
% end
% W = blkdiag(A{:});
% f = f + U'*W*U;
% g = g + 2*W*U;


%% Normalize by steps
% f = f/h;
% g = g/h;

end