function [f, feq, gradf, gradfeq] = projectConstr(U, contexts)

[dim,k] = size(contexts.obstacles);
joints_dim = size(contexts.refTraj,2);

f = zeros(1,length(U) - dim*k);
gradf = zeros(length(U),length(U) - dim*k);
feq = [];
gradfeq = [];

%% Constraints of demonstration weights
for i = 1:joints_dim
    f(i) = 1-U(i);
    gradf(i,i) = -1;
end


%% Constraints of smooth weights
i = joints_dim +1;
f(i) = -U(i);
gradf(i,i) = -1;


%% Constraints of obstacle direction vector
offset = joints_dim+1;

for j = 1:k
%     wk = U(offset+(j-1)*(dim+2)+3:offset+j*(dim+2),1);
%     feq(j) = 1 - wk'*wk; 
%     gradfeq(offset+(j-1)*(dim+2)+3:offset+j*(dim+2),j) = -2*wk;
    
    f(offset+(j-1)*2+1:offset+j*2) = -U(offset+(j-1)*(dim+2)+1:offset+(j-1)*(dim+2)+2);
    gradf(offset+(j-1)*(dim+2)+1:offset+(j-1)*(dim+2)+2,offset+(j-1)*2+1:offset+j*2) = -diag([1,1]);
end

%% Constraints for boarder feature & safety feature
offset_u = offset+k*(dim+2);
offset_f = offset+k*2;

f(offset_f+1) = -U(offset_u+1);
gradf(offset_u+1,offset_f+1) = -1;

f(offset_f+2) = -U(offset_u+2);
gradf(offset_u+2,offset_f+2) = -1;

end