function [f] = regretLoss(w, refTraj, obstacles, optTraj)

[T, dim] = size(refTraj.y_pred);
params.q = w(1:dim);
params.r = zeros(1,2);
% params.r = w(dim+1:dim*2);
params.k = w(dim*2+1:end);

[y_adapted, score] = generateTraj(refTraj, obstacles, params)

f = 0;
for i = 1:T
    f = f + sum((optTraj(i,:)-y_adapted(i,:)).^2);
end