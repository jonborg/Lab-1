function [ theta ] = pro2( pose )
%   pro2 : Computes angles of the hand of a 6-dof arm-robot (should do it...)
%   Receives variable pose of dimensions [1 6] and using moving axes
%   rotation matrixes, returns a matrix with the various combinations of angles
Rzyx=[cos(pose(4))*cos(pose(5)) cos(pose(4))*sin(pose(5))*sin(pose(6))-sin(pose(4))*cos(pose(6))  cos(pose(4))*sin(pose(5))*cos(pose(6))+sin(pose(4))*sin(pose(6));
        sin(pose(4))*cos(pose(5)) sin(pose(4))*sin(pose(5))*sin(pose(6))+cos(pose(4))*cos(pose(6)) sin(pose(4))*sin(pose(5))*cos(pose(6))-cos(pose(4))*sin(pose(6)); 
        -sin(pose(5)) cos(pose(5))*sin(pose(6)) cos(pose(5))*cos(pose(6))];
P=[pose(1) pose(2) pose(3)]';
Tbt=[Rzyx P;0 0 0 1];

theta11=atan2(pose(2),pose(1));
theta12=atan2(pose(2),pose(1))-pi;

K1= ((pose(1)^2)*(cos(theta11)^2)+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);
K2= ((pose(1)^2)*(cos(theta12)^2)+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);

if(0.04^2+0.195^2-K1^2 > 0)
    theta311=atan2(0.04,0.195)-atan2(K1,sqrt(0.04^2+0.195^2-K1^2));
    theta312=atan2(0.04,0.195)-atan2(K1,-sqrt(0.04^2+0.195^2-K1^2));
else
    print('Invalid point.');
    return;
end
theta321=atan2(0.04,0.195)-atan2(K2,sqrt(0.04^2+0.195^2-K2^2));
theta322=atan2(0.04,0.195)-atan2(K2,-sqrt(0.04^2+0.195^2-K2^2));

%theta2...

theta411=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),Tbt(1,3)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))+...
    Tbt(2,3)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))+Tbt(3,3)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311)));

theta412=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),Tbt(1,3)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))+...
    Tbt(2,3)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))+Tbt(3,3)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312)));

theta421=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),Tbt(1,3)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))+...
    Tbt(2,3)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))+Tbt(3,3)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321)));

theta422=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),Tbt(1,3)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))+...
    Tbt(2,3)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))+Tbt(3,3)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322)));


end

