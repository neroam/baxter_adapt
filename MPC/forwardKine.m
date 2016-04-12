function [pose, rotationMat, grad] = forwardKine(joints)

DH = zeros(7:3);

% d, a, alpha
DH(1,:) = [0.27, 0.069, -pi/2];
DH(2,:) = [0, 0, pi/2];  
DH(3,:) = [0.102+0.262, 0.069, -pi/2];
DH(4,:) = [0,0, pi/2];
DH(5,:) = [0.104+0.271, 0.01, -pi/2];
DH(6,:) = [0,0,pi/2];
DH(7,:) = [0.275, 0, 0];

Base = [cos(pi/4)   -sin(pi/4)  0   .064614;
        sin(pi/4)   cos(pi/4)   0   .25858;
        0           0           1   .119;
        0           0           0   1];
    
end_effector = Base;
      
%offset
joints(2) = joints(2) + pi/2;

grad = zeros(4,4,7);
A = zeros(4,4,7);
gradA = zeros(4,4,7);

for i = 1:7
    theta = joints(i);
    d = DH(i,1);
    a = DH(i,2);
    alpha = DH(i,3);
    A(:,:,i) = [cos(theta)   -sin(theta)*cos(alpha)      sin(theta)*sin(alpha)   a*cos(theta);    
           sin(theta)   cos(theta)*cos(alpha)       -cos(theta)*sin(alpha)  a*sin(theta);
           0            sin(alpha)                  cos(alpha)              d;
           0            0                           0                       1];
    gradA(:,:,i) = [-sin(theta)    -cos(theta)*cos(alpha)  cos(theta)*sin(alpha) -a*sin(theta);
           cos(theta)     -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
           0              0                       0                       0;
           0              0                       0                       0];
    
    end_effector = end_effector*A(:,:,i);
   
end

for i = 1:7
    tmpA = Base;
    for j = 1:7
        if i == j
            tmpA = tmpA*gradA(:,:,j);
        else tmpA = tmpA*A(:,:,j);
        end
    end
    grad(:,:,i) = tmpA;
end

pose = zeros(7,1);
pose(1:3) = end_effector(1:3,4);
rotationMat = end_effector(1:3,1:3);
qw = sqrt(1+end_effector(1,1)+end_effector(2,2)+end_effector(3,3))/2;
qx = (end_effector(3,2) - end_effector(2,3))/(4*qw);
qy = (end_effector(1,3) - end_effector(3,1))/(4*qw);
qz = (end_effector(2,1) - end_effector(1,2))/(4*qw);
pose(4:7) = [qx,qy,qz,qw]';


end