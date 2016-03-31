function [ y_adapted, cost ] = generateTraj(contexts, weights, config)
% Generate adapted trajectory

%% Contexts extraction
y_ref = contexts.refTraj;
[T, dim] = size(y_ref); 

%%% Normalization of Variance
y_refVar = contexts.refVar;
for i = 1:dim
%     y_refVar(:,i) = 1 ./ exp(T * contexts.refVar(:,i) / sum(contexts.refVar(:,i), 1));
    y_refVar(:,i) =  exp(-contexts.refVar(:,i));
end


%% MPC configuration
%%% Linear Dynamics for adaptation
%%% x(t+1) = Ax(t) + Bu(t); 
%%% y(t) =  Cx(t)
A = diag(0.9*ones(dim,1));
B = diag(ones(dim,1));
C = diag(ones(dim,1));

%%% Config
h = config.horizon;
step = config.step;
umax = config.umax;


%%% Initialization
y_adapted = zeros(T, dim);
x = zeros(T, dim);
y_adapted(1,:) = y_ref(1,:);
x(1,:) = (inv(C)*y_adapted(1,:)')';
cost = 0;

%% Optimize Trajectory
%%% Optimization parameters
contextOpt = contexts;
contextOpt.terminal = 0;

for i = 1:step:T
    contextOpt.xt = x(i,:)';
    contextOpt.yt = y_adapted(i,:)';
    if i+h >= T
        h = T-i;
        contextOpt.terminal = 1;
    end
    
    contextOpt.horizon = h;
    contextOpt.refTraj = y_ref(i+1:i+h,:);
    contextOpt.refVar = y_refVar(i+1:i+h,:);
    
    %%% Output Predicts
    Px = C*A;
    Pu = C*B;
    
    P = C;
    for k=1:h;
        Puterm = P*B;
        for j=k:h;
            vrow=(j-1)*dim+1:j*dim;
            vcol=(j-k)*dim+1:(j-k+1)*dim;
            Pu(vrow,vcol)=Puterm;
        end
        P=P*A;
        vrow=(k-1)*dim+1:k*dim;
        Px(vrow,1:dim) = P;
    end
    contextOpt.Px = Px;
    contextOpt.Pu = Pu;
    
    %%% Initial
    U0 = zeros(dim*h,1); 
    LB = zeros(dim*h,1) - umax;
    UB = zeros(dim*h,1) + umax;
    
    
    %%% Using fmincon
    options = optimoptions(@fmincon,...
    'GradObj','on','GradConstr','on');
% ,'Algorithm','interior-point',...
%     ,'Hessian','user-supplied','HessFcn',@(U,lambda)quadHess(U,lambda,contextOpt,weights, config)...
%     options = optimoptions(@fmincon,'Algorithm','interior-point');
    fun = @(U)quadObj(U,contextOpt,weights,config);
    nonlconstr = @(U)quadConstr(U,contextOpt,config);

    [Ut,Jt,eflag,output,lambda] = fmincon(fun,U0,...
        [],[],[],[],LB,UB,nonlconstr,options);


    for j = 1:min(step,h)
        x(i+j,:) = (A*x(i+j-1,:)' + B*Ut(dim*(j-1)+1:dim*j,1))';
        y_adapted(i+j,:) = (C*x(i+j,:)')';
    end
    
    %%TODO: fix the score correctly.
    cost = cost + Jt;
    fprintf('Finished Decision Step: %d\n',i);
end

%% Trajectory tail
i = i - step;
for j = step+2:h
    x(i+j,:) = (A*x(i+j-1,:)' + B*Ut(dim*(j-1)+1:dim*j,1))';
    y_adapted(i+j,:) = (C*x(i+j,:)')';
end

end

