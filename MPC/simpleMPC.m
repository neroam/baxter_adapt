% function Y = simpleMPC(refTraj)
refTraj = y_predProMPsMat';
refVar = y_predVar';

%% x(t+1) = Ax(t) + Bx(t); y(t) = Cx(t)
%%%  
A = diag([0.9 0.9]);
B = diag([1 1]);
C = diag([1 1]);
% A = 0.9;
% B = 1;
% C = 1;

nh = 99;

%% Output Predicts
Px=C*A;  Pu=C*B;  P=C;
nx=size(A,1); % num of state
nu=size(B,2); % num of input
ny = size(C,1); % num of output

for i=1:nh;
   
   Puterm = P*B;
   for j=i:nh;
         vrow=(j-1)*ny+1:j*ny;
         vcol=(j-i)*nu+1:(j-i+1)*nu;
         Pu(vrow,vcol)=Puterm;
   end
   P=P*A;
   vrow=(i-1)*ny+1:i*ny;
   Px(vrow,1:nx) = P;
end

%% Objective function
%%% J = Y'QY + U'RU ; Y = PxX + PuU
Q = diag([1 1]);
R = diag([0. 0.]);
% Q = 1;
% R = 0.1;

%% Optimize Trajectory
obs = [0.4; 0.8];
% obs = 0.5;

LB = zeros(nu*nh,1) - 2;
UB = zeros(nu*nh,1) + 2;

T = size(refTraj,1);
Y = zeros(T,ny);
X = zeros(T,nx);
Y(1,:) = refTraj(1,:);
X(1,:) = (inv(C)*Y(1,:)')';
sumJ = 0;

% Normalization of Variance
norm_refVar = refVar;
for i = 1:size(refVar, 2)
%     norm_refVar(:,i) = ones(T,1);
    norm_refVar(:,i) = 1 ./ exp(T * refVar(:,i) / sum(refVar(:,i), 1));

end

for i = 1:T-nh
    xt = X(i,:)';
    funObj = @(U) costFun(xt, U, Px, Pu, Q, R, nh, refTraj(i+1:i+nh,:), obs, norm_refVar(i+1:i+nh,:), i == T-nh);
    [U,J] = minConf_TMP(funObj,zeros(nu*nh+2,1),[LB; -inf; -inf], [UB ; inf; inf;]);
    X(i+1,:) = (A*xt + B*U(1:nu,1))';
    Y(i+1,:) = (C*X(i+1,:)')';
    sumJ = sumJ + J;
end

% Ensure the end location is the same as the target positio

for i = 1:nh-1
    xt = X(T-nh+i,:)';
    X(T-nh+i+1,:) = (A*xt + B*U(1+nu*i:nu*(i+1),1))';
    Y(T-nh+i+1,:) = C*X(T-nh+i+1,:)';
end

% 
% figure;
% plot(refTraj');
% hold all;
% plot(Y');
figure;
plot(refTraj(:,1)', refTraj(:,2)');
hold all;
plot(Y(:,1)', Y(:,2)');
scatter(obs(1),obs(2), 100, 'xb', 'linewidth',4);
% 
% figure;
% plot(refTraj(:,1)');
% hold all;
% plot(Y(:,1)');
% plot(obs(1));

% end
