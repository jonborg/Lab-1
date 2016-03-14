function [ theta ] = pro2( pose )
%   pro2 : Computes angles of the hand of a 6-dof arm-robot (should do it...)
%   Receives variable pose of dimensions [1 6] and using moving axes
%   rotation matrixes, returns a matrix with the various combinations of angles
Rzyx=[cos(pose(4))*cos(pose(5)) cos(pose(4))*sin(pose(5))*sin(pose(6))-sin(pose(4))*cos(pose(6))  cos(pose(4))*sin(pose(5))*cos(pose(6))+sin(pose(4))*sin(pose(6));
        sin(pose(4))*cos(pose(5)) sin(pose(4))*sin(pose(5))*sin(pose(6))+cos(pose(4))*cos(pose(6)) sin(pose(4))*sin(pose(5))*cos(pose(6))-cos(pose(4))*sin(pose(6)); 
        -sin(pose(5)) cos(pose(5))*sin(pose(6)) cos(pose(5))*cos(pose(6))];
P=[pose(1) pose(2) pose(3)]';
Tbt=[Rzyx P;0 0 0 1];

theta11=atan2(pose(2),pose(1))
theta12=atan2(pose(2),pose(1))-pi

K1= ((pose(1)^2)*(cos(theta11)^2)+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);
K2= ((pose(1)^2)*(cos(theta12)^2)+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);

if(0.04^2+0.195^2-K1^2 > 0)
    theta311=atan2(0.04,0.195)-atan2(K1,sqrt(0.04^2+0.195^2-K1^2));
    theta312=atan2(0.04,0.195)-atan2(K1,-sqrt(0.04^2+0.195^2-K1^2));
else
    print('Invalid point.');
    return;
end
theta321=atan2(0.04,0.195)-atan2(K2,sqrt(0.04^2+0.195^2-K2^2))
theta322=atan2(0.04,0.195)-atan2(K2,-sqrt(0.04^2+0.195^2-K2^2))

%theta2...

theta411=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),Tbt(1,3)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))+...
    Tbt(2,3)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))+Tbt(3,3)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311)));

theta412=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),Tbt(1,3)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))+...
    Tbt(2,3)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))+Tbt(3,3)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312)));

theta421=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),Tbt(1,3)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))+...
    Tbt(2,3)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))+Tbt(3,3)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321)));

theta422=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),Tbt(1,3)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))+...
    Tbt(2,3)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))+Tbt(3,3)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322)));

a11=cos(theta411)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))-sin(theta11)*sin(theta411);
a12=cos(theta412)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))-sin(theta11)*sin(theta412);
a21=cos(theta421)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))-sin(theta12)*sin(theta421);
a22=cos(theta422)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))-sin(theta12)*sin(theta422);

b11=cos(theta411)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))+cos(theta11)*sin(theta411);
b12=cos(theta412)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))+cos(theta11)*sin(theta412);
b21=cos(theta421)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))+cos(theta12)*sin(theta421);
b22=cos(theta422)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))+cos(theta12)*sin(theta422);

c11=cos(theta411)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311));
c12=cos(theta412)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312));
c21=cos(theta421)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321));
c22=cos(theta422)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322));

g11=-cos(theta11)*cos(theta211)*sin(theta311)-cos(theta11)*sin(theta211)*cos(theta311);
g12=-cos(theta11)*cos(theta212)*sin(theta312)-cos(theta11)*sin(theta212)*cos(theta312);
g21=-cos(theta12)*cos(theta221)*sin(theta321)-cos(theta12)*sin(theta221)*cos(theta321);
g22=-cos(theta12)*cos(theta222)*sin(theta322)-cos(theta12)*sin(theta222)*cos(theta322);

h11=-sin(theta11)*cos(theta211)*sin(theta311)-sin(theta11)*sin(theta211)*cos(theta311);
h12=-sin(theta11)*cos(theta212)*sin(theta312)-sin(theta11)*sin(theta212)*cos(theta312);
h21=-sin(theta12)*cos(theta221)*sin(theta321)-sin(theta12)*sin(theta221)*cos(theta321);
h22=-sin(theta12)*cos(theta222)*sin(theta322)-sin(theta12)*sin(theta222)*cos(theta322);

i11=-sin(theta211)*cos(theta311)+cos(theta211)*cos(theta311);
i12=-sin(theta212)*cos(theta312)+cos(theta212)*cos(theta312);
i21=-sin(theta221)*cos(theta321)+cos(theta221)*cos(theta321);
i22=-sin(theta222)*cos(theta322)+cos(theta222)*cos(theta322);

theta511=atan2(-(Tbt(1,3)*a11+Tbt(2,3)*b11+Tbt(3,3)*c11),Tbt(1,3)*g11+Tbt(2,3)*h11+Tbt(3,3)*i11);
theta512=atan2(-(Tbt(1,3)*a12+Tbt(2,3)*b12+Tbt(3,3)*c12),Tbt(1,3)*g12+Tbt(2,3)*h12+Tbt(3,3)*i12);
theta521=atan2(-(Tbt(1,3)*a21+Tbt(2,3)*b21+Tbt(3,3)*c21),Tbt(1,3)*g21+Tbt(2,3)*h21+Tbt(3,3)*i21);
theta522=atan2(-(Tbt(1,3)*a22+Tbt(2,3)*b22+Tbt(3,3)*c22),Tbt(1,3)*g22+Tbt(2,3)*h22+Tbt(3,3)*i22);

j11=cos(theta511)*a11+sin(theta511)*g11;
j12=cos(theta512)*a12+sin(theta512)*g12;
j21=cos(theta521)*a21+sin(theta521)*g21;
j22=cos(theta522)*a22+sin(theta522)*g22;

k11=cos(theta511)*b11+sin(theta511)*h11;
k12=cos(theta512)*b12+sin(theta512)*h12;
k21=cos(theta521)*b21+sin(theta521)*h21;
k22=cos(theta522)*b22+sin(theta522)*h22;

l11=cos(theta511)*c11+sin(theta511)*i11;
l12=cos(theta512)*c12+sin(theta512)*i12;
l21=cos(theta521)*c21+sin(theta521)*i21;
l22=cos(theta522)*c22+sin(theta522)*i22;

p11=sin(theta411)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))-sin(theta11)*cos(theta411);
p12=sin(theta412)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))-sin(theta11)*cos(theta412);
p21=sin(theta421)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))-sin(theta12)*cos(theta421);
p22=sin(theta422)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))-sin(theta12)*cos(theta422);

q11=sin(theta411)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))+cos(theta11)*cos(theta411);
q12=sin(theta412)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))+cos(theta11)*cos(theta412);
q21=sin(theta421)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))+cos(theta12)*cos(theta421);
q22=sin(theta422)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))+cos(theta12)*cos(theta422);

r11=sin(theta411)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311));
r12=sin(theta412)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312));
r21=sin(theta421)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321));
r22=sin(theta422)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322));

theta611=atan2(-(Tbt(1,3)*p11+Tbt(2,3)*q11+Tbt(3,3)*r11),Tbt(1,1)*j11+Tbt(2,1)*k11+Tbt(3,1)*l11);
theta612=atan2(-(Tbt(1,3)*p12+Tbt(2,3)*q12+Tbt(3,3)*r12),Tbt(1,1)*j12+Tbt(2,1)*k12+Tbt(3,1)*l12);
theta621=atan2(-(Tbt(1,3)*p21+Tbt(2,3)*q21+Tbt(3,3)*r21),Tbt(1,1)*j21+Tbt(2,1)*k21+Tbt(3,1)*l21);
theta622=atan2(-(Tbt(1,3)*p22+Tbt(2,3)*q22+Tbt(3,3)*r22),Tbt(1,1)*j22+Tbt(2,1)*k22+Tbt(3,1)*l22);

%soluções alternativas para theta4,5,6...
end

