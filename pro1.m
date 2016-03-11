function [pose] = pro1(teta)
%   pro1 : Computes the pose of the hand of a 6-dof arm-robot
%   Receives variable teta of dimensions [1 6] and using moving axes
%   rotation matrixes, returns vector pose of imensions [1 6] that gives
%   (x,y,z,alpha,beta,gamma) parameters of the hand
%
%   teta=[0 pi/4 -pi/4 0 0 anything] should give a similar position to the
%   figure1 in the lab assignment 
for i=1:6
    sin_check(sin(teta(i)),i);
    sin_check(cos(teta(i)),i);
end
R01=[cos(teta(1)) -sin(teta(1)) 0; sin(teta(1)) cos(teta(1)) 0; 0 0 1]; %rotação em torno de z
T01=[R01 zeros(3,1);0 0 0 1];
dof(:,1)=[0 0 0 1]';

R12=[1 0 0; 0 cos(teta(2)) -sin(teta(2)); 0 sin(teta(2)) cos(teta(2))]; %rotação em torno de x
P12=[0 0 99*10^-3]';
T12=[R12 P12;0 0 0 1];
dof(:,2)=T01*T12*[0 0 0 1]';

R23=[1 0 0; 0 cos(teta(3)) -sin(teta(3)); 0 sin(teta(3)) cos(teta(3))]; %rotação em torno de x
P23=[0 0 120*10^-3]';
T23=[R23 P23;0 0 0 1];
dof(:,3)=T01*T12*T23*[0 0 0 1]';

R34=[cos(teta(4)) 0 sin(teta(4)); 0 1 0; -sin(teta(4)) 0 cos(teta(4))]; %rotação em torno de y
P34=[0 0 30*10^-3]'; %necessário verificar
T34=[R34 P34;0 0 0 1];
dof(:,4)=T01*T12*T23*T34*[0 0 0 1]';

R45=[1 0 0; 0 cos(teta(5)) -sin(teta(5)); 0 sin(teta(5)) cos(teta(5))]; %rotação em torno de x
P45=[0 160*10^-3 0]'; %necessário verificar
T45=[R45 P45;0 0 0 1];
dof(:,5)=T01*T12*T23*T34*T45*[0 0 0 1]';

R56=[cos(teta(6)) 0 sin(teta(6)); 0 1 0; -sin(teta(6)) 0 cos(teta(6))]; %rotação em torno de y
P56=[0 20*10^-3 0]'; %ampliado para testes
T56=[R56 P56;0 0 0 1];
dof(:,6)=T01*T12*T23*T34*T45*T56*[0 0 0 1]';
T06=T01*T12*T23*T34*T45*T56;

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

