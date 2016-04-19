function [ y_adapted, joints, cost ] = jointsTraj(contexts, weights, config)
% Generate adapted trajectory

global y_adapt;
global joints_adapt;

%% Contexts extraction
y_ref = contexts.refTraj;
[T, dim] = size(y_ref); 

%%% Normalization of Variance
y_refVar = contexts.refVar;
for i = 1:dim
%     y_refVar(:,i) = 1 ./ exp(T * contexts.refVar(:,i) / sum(contexts.refVar(:,i), 1));
%     y_refVar(:,i) =  exp(-contexts.refVar(:,i));
    y_refVar(:,i) = 1;
end

joints_dim = 7;
%% MPC configuration
%%% Linear Dynamics for adaptation
%%% x(t+1) = Ax(t) + Bu(t); 
%%% y(t) =  Cx(t)
A = diag(0.9*ones(joints_dim,1));
B = diag(ones(joints_dim,1));
C = diag(ones(joints_dim,1));

%%% Config
h = config.horizon;
step = config.step;
umax = config.umax;

%%% Initialization
y_adapted = zeros(T, 7);
y_adapted(1,:) = forwardKine(contexts.start_joints);

joints = zeros(T, joints_dim);
x = zeros(T, joints_dim);
x(1,:) = contexts.start_joints;
joints(1,:) = contexts.start_joints;
cost = 0;

%% Optimize Trajectory
%%% Optimization parameters
contextOpt = contexts;
contextOpt.terminal = 0;

for i = 1:step:T
    contextOpt.xt = x(i,:)';
    contextOpt.yt = joints(i,:)';
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
            vrow=(j-1)*joints_dim+1:j*joints_dim;
            vcol=(j-k)*joints_dim+1:(j-k+1)*joints_dim;
            Pu(vrow,vcol)=Puterm;
        end
        P=P*A;
        vrow=(k-1)*joints_dim+1:k*joints_dim;
        Px(vrow,1:joints_dim) = P;
    end
    contextOpt.Px = Px;
    contextOpt.Pu = Pu;
    
    %%% Initial
    U0 = zeros(joints_dim*h,1); 
    LB = zeros(joints_dim*h,1) - umax;
    UB = zeros(joints_dim*h,1) + umax;
    
    
    %%% Using fmincon
    options = optimoptions(@fmincon,...
    'MaxFunEvals',10000,'MaxIter',2000,'TolFun', 1e-10,'GradConstr','on','GradObj','on');
% 'GradConstr','on',
% ,'Algorithm','interior-point',...
%     ,'Hessian','user-supplied','HessFcn',@(U,lambda)quadHess(U,lambda,contextOpt,weights, config)...
%     options = optimoptions(@fmincon,'Algorithm','interior-point');
    fun = @(U)jointsObj(U,contextOpt,weights,config);
    nonlconstr = @(U)jointsConstr(U,contextOpt,config);

    [Ut,Jt,eflag,output,lambda] = fmincon(fun,U0,...
        [],[],[],[],LB,UB,nonlconstr,options);


    for j = 1:min(step,h)
        x(i+j,:) = (A*x(i+j-1,:)' + B*Ut(joints_dim*(j-1)+1:joints_dim*j,1))';
        joints(i+j,:) = (C*x(i+j,:)')';
        [pose, ~] = forwardKine(joints(i+j,:)');
        y_adapted(i+j,:) = pose';
    end
    
    %%TODO: fix the score correctly.
    cost = cost + Jt;
    fprintf('Finished Decision Step: %d\n',i);
end

%% Trajectory tail
i = i - step;
for j = step+2:h
    x(i+j,:) = (A*x(i+j-1,:)' + B*Ut(joints_dim*(j-1)+1:joints_dim*j,1))';
    joints(i+j,:) = (C*x(i+j,:)')';
    [pose, ~,~] = forwardKine(joints(i+j,:)');
    y_adapted(i+j,:) = pose';
end

joints_adapt = joints;
y_adapt = y_adapted;

writeTrajectoryJoints(contexts.filename, joints');

end

