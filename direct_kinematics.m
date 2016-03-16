function [pose] = direct_kinematics(theta)
%   direct_kinematics : Computes the pose of the hand of a 6-dof arm-robot 
%   Receives variable theta of dimensions [1 6] and using moving axes
%   rotation matrixes, returns vector pose of imensions [1 6] that gives
%   (x,y,z,alpha,beta,gamma) parameters of the hand, using an Euler angles
%   Z-Y-X convention.
%   An illustrative figure of the robot arms is also shown.

R01=[cos(theta(1)) -sin(theta(1)) 0; sin(theta(1)) cos(theta(1)) 0; 0 0 1]; 
T01=[R01 [0 0 99*10^-3]';0 0 0 1];
dof(:,1)=[0 0 0 1]';

R12=[cos(theta(2)) -sin(theta(2)) 0; 0 0 -1; sin(theta(2)) cos(theta(2)) 0]; 
P12=[0 0 0]';
T12=[R12 P12;0 0 0 1];
dof(:,2)=T01*T12*[0 0 0 1]';

R23=[cos(theta(3)) -sin(theta(3)) 0; sin(theta(3)) cos(theta(3)) 0 ; 0 0 1]; 
P23=[120*10^-3 0 0]';
T23=[R23 P23;0 0 0 1];
dof(:,3)=T01*T12*T23*[0 0 0 1]';

R34=[cos(theta(4)) -sin(theta(4)) 0; 0 0 1; -sin(theta(4)) -cos(theta(4)) 0]; 
P34=[40*10^-3 0 0]'; 
T34=[R34 P34;0 0 0 1];
dof(:,4)=T01*T12*T23*T34*[0 0 0 1]';

R4A=eye(3); 
P4A=[0 0 195*10^-3]'; 
T4A=[R4A P4A;0 0 0 1];

RA5=[cos(theta(5)) -sin(theta(5)) 0; 0 0 -1; sin(theta(5)) cos(theta(5)) 0]; 
PA5=[0 0 0]'; 
TA5=[RA5 PA5;0 0 0 1];
dof(:,5)=T01*T12*T23*T34*T4A*TA5*[0 0 0 1]';

R56=[cos(theta(6)) -sin(theta(6)) 0; 0 0 1; -sin(theta(6)) -cos(theta(6)) 0]; 
P56=[0 0 0]'; 
T56=[R56 P56;0 0 0 1];
dof(:,6)=T01*T12*T23*T34*T4A*TA5*T56*[0 0 0 1]';

T06=T01*T12*T23*T34*T4A*TA5*T56;

plot3(dof(1,:),dof(2,:),dof(3,:),'Marker','o');
title('Robot Arm');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-0.3 0.4 -0.3 0.4 -0.3 0.4]);

beta=atan2(-T06(3,1),sqrt((T06(1,1))^2+(T06(2,1))^2));
if(cos(beta)~=0)
    alpha=atan2(T06(2,1)/cos(beta),T06(1,1)/cos(beta));
    gamma=atan2(T06(3,2)/cos(beta),T06(3,3)/cos(beta));
else
    alpha=0;
    if(beta>0)
        gamma=atan2(T06(1,2),T06(2,2));
    else
        gamma=-atan2(T06(1,2),T06(2,2));
    end
end
pose=[T06(1,4),T06(2,4),T06(3,4),alpha,beta,gamma];

end

