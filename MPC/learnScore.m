function [ Wt, Jt ] = learnScore( dataTraj, refTraj, obstacles, y)
% learn score weights from data

[T, dim] = size(refTraj.y_pred);

obst = repmat(obstacles', T, 1);
minD_new = min(sum((dataTraj-obst).^2,2));
minD = min(sum((y-obst).^2,2));

% initW = -log(0.2)/minD;
initW = 500;

W0 = [1;1;initW;initW;0;0];
LB = zeros(dim*3,1);
UB = zeros(dim*3,1) + 1000;
options.numDiff = 1;
options.maxIter = 250;

funObj = @(W) regretLoss(W,refTraj,obstacles, dataTraj);
[Wt,Jt] = minConf_TMP(funObj,W0, LB, UB, options);

Wt = Wt';

end

