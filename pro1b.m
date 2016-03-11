function [pose] = pro1b(teta)
%pro1 : Computes the pose of the hand of a 6-dof arm-robot (should do it...)
%   Receives variable teta of dimensions [1 6] and using moving axes
%   rotation matrixes, returns vector pose of imensions [1 6] that gives
%   (x,y,z,alpha,beta,gamma) parameters of the hand
%
%   teta=[0 pi/4 -pi/4 0 0 anything] should give a similar position to the
%   figure1 in the lab assignment 

R01=[cos(teta(1)) -sin(teta(1)) 0; sin(teta(1)) cos(teta(1)) 0; 0 0 1]; %rotação em torno de z
T01=[R01 [0 0 99*10^-3]';0 0 0 1]
dof(:,1)=[0 0 0 1]';

R12=[cos(teta(2)) -sin(teta(2)) 0; 0 0 -1; sin(teta(2)) cos(teta(2)) 0]; %rotação em torno de x
P12=[0 0 0]';
T12=[R12 P12;0 0 0 1];
dof(:,2)=T01*T12*[0 0 0 1]';

R23=[cos(teta(3)) -sin(teta(3)) 0; sin(teta(3)) cos(teta(3)) 0 ; 0 0 1]; %rotação em torno de x
P23=[120*10^-3 0 0]';
T23=[R23 P23;0 0 0 1];
dof(:,3)=T01*T12*T23*[0 0 0 1]';

R34=[cos(teta(4)) -sin(teta(4)) 0; 0 0 1; -sin(teta(4)) -cos(teta(4)) 0]; %rotação em torno de y
P34=[40*10^-3 0 0]'; %necessário verificar
T34=[R34 P34;0 0 0 1];
dof(:,4)=T01*T12*T23*T34*[0 0 0 1]';

R4A=eye(3); %rotação em torno de y
P4A=[0 0 195*10^-3]'; %necessário verificar
T4A=[R4A P4A;0 0 0 1];

RA5=[cos(teta(5)) -sin(teta(5)) 0; 0 0 -1; sin(teta(5)) cos(teta(5)) 0]; %rotação em torno de x
PA5=[0 0 0]'; %necessário verificar
TA5=[RA5 PA5;0 0 0 1];
dof(:,5)=T01*T12*T23*T34*T4A*TA5*[0 0 0 1]';

R56=[cos(teta(6)) -sin(teta(6)) 0; 0 0 1; -sin(teta(6)) -cos(teta(6)) 0]; %rotação em torno de y
P56=[0 0 0]'; %ampliado para testes
T56=[R56 P56;0 0 0 1];
dof(:,6)=T01*T12*T23*T34*T4A*TA5*T56*[0 0 0 1]'

T06=T01*T12*T23*T34*T4A*TA5*T56

plot3(dof(1,:),dof(2,:),dof(3,:),'Marker','o');
title('Robot Arm');
xlabel('x');
ylabel('y');
zlabel('z');
axis([-0.3 0.4 -0.3 0.4 -0.3 0.4]);


beta=atan2(sqrt((T06(1,3))^2+(T06(2,3))^2),T06(3,3));
if(sin(beta)>0)
    gama=atan2(T06(3,2),-T06(3,1));
    alpha=atan2(T06(2,3),T06(1,3));
else
    gama=atan2(-T06(3,2),T06(3,1));
    alpha=atan2(-T06(2,3),-T06(1,3));
end

pose=[T06(1,4),T06(2,4),T06(3,4),alpha,beta,gama];

end

