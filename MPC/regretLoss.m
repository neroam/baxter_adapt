function [f] = regretLoss(w, refTraj, obstacles, optTraj)

[T, dim] = size(refTraj.y_pred);
w(end-dim+1:end) = 0;

[y_adapted, ~] = generateTraj(refTraj, obstacles, w, 1);

hold on;
plot(optTraj(:,1), optTraj(:,2));
hold off;
% figure;
% plot(sum((optTraj - y_adapted).^2, 2));
% figure;
% plot(optTraj(:,1));
% hold on;
% plot(y_adapted(:,1));
% figure;
% plot(optTraj(:,2));
% hold on;
% plot(y_adapted(:,2));

% f = HausdorffDist(optTraj, y_adapted);

f = 0;
for i = 1:T
    f = f + sum((optTraj(i,:)-y_adapted(i,:)).^2);
end