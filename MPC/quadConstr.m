function [f, feq, gradf, gradfeq] = quadConstr(U, contexts, config)
% Constraints of obstacles

%% Contexts extraction
x = contexts.xt;
Px = contexts.Px;
Pu = contexts.Pu;
[h, dim] = size(contexts.refTraj);
obstacles = contexts.obstacles;

%% Configuration
d = config.dmin;

%% Predict future outputs
Y = Px*x + Pu*U;

k = size(obstacles,2);

f = zeros(1,h*k);
gradf = zeros(length(U),h*k);

%% Avoid obstacles constraints
for j = 1:k
    obst = obstacles(:,j);
    for i = 1:h
        yt = Y((i-1)*dim+1:i*dim,1);
        f((j-1)*h+i) = d - (yt-obst)'*(yt-obst);
        gradf(:,(j-1)*h+i) = -2*Pu((i-1)*dim+1:i*dim,:)'*(yt-obst);
    end
end

feq = [];
gradfeq = [];

%% Terminal state constraint
if contexts.terminal
    yh = Y((h-1)*dim+1:h*dim,1);
    feq = (yh-contexts.refTraj(end,:)')'*(yh-contexts.refTraj(end,:)');
    gradfeq = 2*Pu((h-1)*dim+1:h*dim,:)'*(yh-contexts.refTraj(end,:)');

end


end