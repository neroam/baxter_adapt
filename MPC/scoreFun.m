function [f, g] = scoreFun(U, par)
%U (nu*h+1, 1)

%par:
%x  (nx, 1)
%Px (ny*h, nx)
%Pu (ny*h, nu*h)
%Q  (ny, ny)
%R  (nu, nu)
%K  (ny, ny)
%refTraj (nh, ny)
%refVar (nh, ny)
%obst (ny, 1)

dim = par.dim;
h = par.h;
x = par.xt;
Px = par.Px;
Pu = par.Pu;
Q = par.Q;
R = par.R;
K = par.K;
refTraj = par.refTraj;
refVar = par.refVar;
obst = par.obst;

%% Predict future outputs
Y = Px*x + Pu*U;


%% Score for avoiding obstacle
refTraj = reshape(refTraj',[],1);
refVar = reshape(refVar',[],1);

A = cell(1, h);
B = cell(1, h);
for i = 1:h
%     A{i} = Q * diag(refVar((i-1)*dim+1:i*dim,1));
    A{i} = Q;
    B{i} = R;
end
Q = blkdiag(A{:});
R = blkdiag(B{:});

f_obst = 0;
g_obst = zeros(dim*h,1);
% cost and gradient in exponential
% for i = 1:h
%     yt = Y((i-1)*dim+1:i*dim,1);
% %     if (yt-obst)'*(yt-obst) < 0.01
%         f_obst = f_obst + exp(-(yt-obst)'*K*(yt-obst)) - exp(-([0; yt(2) + 1])'*diag([1 1])*([0; yt(2) + 1]));
%         g_obst = g_obst - exp(-(yt-obst)'*K*(yt-obst))*2*Pu((i-1)*dim+1:i*dim,:)'*K*(yt-obst) + exp(-([0; yt(2) + 1])'*diag([1 1])*([0; yt(2) + 1]))*2*Pu((i-1)*dim+1:i*dim,:)'*diag([1 1])*([0; yt(2) + 1]);
% %     end
% end

% for i = 1:h
%     yt = Y((i-1)*dim+1:i*dim,1);
% %     if (yt-obst)'*(yt-obst) < 0.01
%         f_obst = f_obst + exp(-(yt-obst)'*K*(yt-obst));
%         g_obst = g_obst - exp(-(yt-obst)'*K*(yt-obst))*2*Pu((i-1)*dim+1:i*dim,:)'*K*(yt-obst);
% %     end
% end

% cost and gradient in 1/d^2
% 
% for i = 1:h
%     yt = Y((i-1)*dim+1:i*dim,1);
%     if (yt-obst)'*(yt-obst) < 0.02
%         f_obst = f_obst + sum(((1./(yt-obst)).^3).*diag(K));
%         g_obst = g_obst - 3*Pu((i-1)*dim+1:i*dim,:)'*((1./(yt-obst)).^4.*diag(K));
%     end
% end

% cost and gradient in 1-x^2
for i = 1:h
    yt = Y((i-1)*dim+1:i*dim,1);
    if (yt-obst)'*(yt-obst) < 0.005
        f_obst = f_obst + (1 - (yt-obst)'*K*(yt-obst))^2;
        g_obst = g_obst - 2*Pu((i-1)*dim+1:i*dim,:)'*K*(yt-obst)*2*(1 - (yt-obst)'*K*(yt-obst));
    end
end

% for i = 1:h
%     yt = Y((i-1)*dim+1:i*dim,1);
%     if (yt-obst)'*(yt-obst) < 0.02
%         f_obst = f_obst + 1 - (yt-obst)'*K*(yt-obst);
%         g_obst = g_obst - 2*Pu((i-1)*dim+1:i*dim,:)'*K*(yt-obst);
%     end
% end


f = (Y-refTraj)'*Q*(Y-refTraj) + U'*R*U + f_obst;
g = 2*Pu'*Q*(Y-refTraj) + 2*R*U + g_obst;
end