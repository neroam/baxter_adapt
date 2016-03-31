function [f, g] = extractFeatures(Y, contexts, weights, config)

% Y (h, dim)

% contexts:
%   Px (dim*h, dim)
%   Pu (dim*h, dim*h)
%   refTraj  (h, dim)
%   refVar  (h, dim)
%   obstacles (dim, k)

% config:
%   dmin (min distance to avoid obstacle)
%   scale (scale for features decaying)

%% Contexts extraction
x = contexts.xt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, dim] = size(contexts.refTraj);
refTraj = reshape(contexts.refTraj',[],1);
refVar = contexts.refVar;
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;
s = config.scale;

f = [];
g = [];

for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);  
    yt_ref = refTraj((i-1)*dim+1:i*dim,1);

    %% Demonstration Feature
    ft = (yt-yt_ref).^2;
    gt = 2*Pu((i-1)*dim+1:i*dim,:)
    

end



f = (Y-refTraj)'*Q*(Y-refTraj) + U'*R*U;
g = 2*Pu'*Q*(Y-refTraj) + 2*R*U;

for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);  
    yt_ref = refTraj((i-1)*dim+1:i*dim,1);
%     f = f + K*(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d);
%     g = g + K*(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d...
%         + exp(-(yt-obst)'*(yt-obst)/d)*(K*Pu((i-1)*dim+1:i*dim,:))';
    f = f - K*(yt-obst)/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s);
    g = g - (K*Pu((i-1)*dim+1:i*dim,:))'/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)...
        + K*(yt-obst)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/(norm(yt-obst)^3)*exp(-(yt-obst)'*(yt-obst)/d*s)...
        - K*(yt-obst)/norm(yt-obst)*exp(-(yt-obst)'*(yt-obst)/d*s)*(-2)*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst)/d*s;
    
end

f = f/h;
g = g/h;

end