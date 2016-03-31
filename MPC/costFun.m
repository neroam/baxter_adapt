function [f, g] = costFun(x, U, Px, Pu, Q, R, nh, refTraj, obst, norm_refVar, isTerm)
%x (nx, 1)
%U (nu*h+1, 1)
%Px (ny*h, nx)
%Pu (ny*h, nu*h)
%Q  (ny, ny)
%R  (nu, nu)
%refTraj (nh, ny)
%obst (ny, 1)
%norm_refVar (nh, ny)

% Ensure end location is same as target location
lambda = zeros(2,1);
if isTerm
    lambda = U(end-1:end,1)
end
U = U(1:end-2,1);
targetLoc = refTraj(end,:)';


%Y (ny*nh, 1)
Y = Px*x + Pu*U;
Ymat = reshape(Y, [], nh);
refTraj = reshape(refTraj',[],1);
norm_refVar = reshape(norm_refVar',[],1);
obst = repmat(obst, 1, nh);
Ydiff = Ymat - obst;

K = 100;

C_e = sum(exp(-K*sum(Ydiff.^2,1)),2);
ny = size(Q, 1);
g_ce = zeros(size(Pu, 2),1);
for i = 0:nh-1
    g_ce = g_ce - exp(-K*sum(Ydiff(:,i+1).^2,1))*2*Pu(i*ny+1:(i+1)*ny, :)'*diag([K, K])*Ydiff(:,i+1);
end

B = cell(1, nh);
C = cell(1, nh);
for i = 1:nh
    B{i} = Q * diag(norm_refVar((i-1)*ny+1:i*ny,1));
    C{i} = R;
end
Q = blkdiag(B{:});
R = blkdiag(C{:});

% constraint at end location
C_end = abs(Y(end-ny+1:end,1) - targetLoc)

f = (Y-refTraj)'*Q*(Y-refTraj) + U'*R*U + C_e + lambda'*C_end;
% g = 2*Pu'*Q*(Y-refTraj) + 2*R*U + g_ce + lambda*2*Pu(end-ny+1:end,:)'*(Y(end-ny+1:end,1) - targetLoc);
g = 2*Pu'*Q*(Y-refTraj) + 2*R*U + g_ce + Pu(end-ny+1:end,:)'*lambda;
g = [g; C_end];
% f = C_e;
% g = g_ce;
end