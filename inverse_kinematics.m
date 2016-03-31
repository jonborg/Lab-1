function [ theta ] = inverse_kinematics( pose )
%   inverse_kinematics : Computes angles of the hand of a 6-dof arm-robot.
%   Receives variable pose of dimensions [1 6], representing (x,y,z,alpha,beta,gamma)
%   according to an Z-Y-X Euler angles convention and returns a [8 6]   
%   matrix with the 8 possible combinations of angles (theta1,...,theta6).
%   If the input point is invalid or if theta5 can have a value k*pi (which
%   means there is a singularity), the output is simply the value -1.
%   If x==y==0, there is another singularity and theta1 can have any value.
Rzyx=[cos(pose(4))*cos(pose(5)) cos(pose(4))*sin(pose(5))*sin(pose(6))-sin(pose(4))*cos(pose(6))  cos(pose(4))*sin(pose(5))*cos(pose(6))+sin(pose(4))*sin(pose(6));
        sin(pose(4))*cos(pose(5)) sin(pose(4))*sin(pose(5))*sin(pose(6))+cos(pose(4))*cos(pose(6)) sin(pose(4))*sin(pose(5))*cos(pose(6))-cos(pose(4))*sin(pose(6)); 
        -sin(pose(5)) cos(pose(5))*sin(pose(6)) cos(pose(5))*cos(pose(6))];
P=[pose(1) pose(2) pose(3)]';
Tbt=[Rzyx P;0 0 0 1];

if(pose(1)==0 && pose(2)==0)
    display('Singularity, theta1 can have any value, not just the ones displayed.')
end

theta11=atan2(pose(2),pose(1));
theta12=atan2(pose(2),pose(1))-pi;

K1= ((cos(theta11)*pose(1)+sin(theta11)*pose(2))^2+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);
K2= ((cos(theta11)*pose(1)+sin(theta11)*pose(2))^2+(pose(3)-0.099)^2-0.12^2-0.04^2-0.195^2)/(2*0.12);

if(0.04^2+0.195^2-K1^2 > 0)
    theta311=atan2(0.04,0.195)-atan2(K1,sqrt(0.04^2+0.195^2-K1^2));
    theta312=atan2(0.04,0.195)-atan2(K1,-sqrt(0.04^2+0.195^2-K1^2));
else
    display('Invalid point.');
    theta=-1;
    return;
end
theta321=atan2(0.04,0.195)-atan2(K2,sqrt(0.04^2+0.195^2-K2^2));
theta322=atan2(0.04,0.195)-atan2(K2,-sqrt(0.04^2+0.195^2-K2^2));

A1=cos(theta11)*pose(1)+sin(theta11)*pose(2);
A2=cos(theta12)*pose(1)+sin(theta12)*pose(2);
B=pose(3)-0.099;
C11=cos(theta311)*0.04-sin(theta311)*0.195+0.12;
C12=cos(theta312)*0.04-sin(theta312)*0.195+0.12;
C21=cos(theta321)*0.04-sin(theta321)*0.195+0.12;
C22=cos(theta322)*0.04-sin(theta322)*0.195+0.12;
D11=sin(theta311)*0.04+cos(theta311)*0.195;
D12=sin(theta312)*0.04+cos(theta312)*0.195;
D21=sin(theta321)*0.04+cos(theta321)*0.195;
D22=sin(theta322)*0.04+cos(theta322)*0.195;

theta211=atan2(C11*B-D11*A1,C11*A1+D11*B);
theta212=atan2(C12*B-D12*A1,C12*A1+D12*B);
theta221=atan2(C21*B-D21*A2,C21*A2+D21*B);
theta222=atan2(C22*B-D22*A2,C22*A2+D22*B);

theta411=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),-(Tbt(1,3)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))+...
    Tbt(2,3)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))+Tbt(3,3)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311))));

theta412=atan2(Tbt(1,3)*sin(theta11)-Tbt(2,3)*cos(theta11),-(Tbt(1,3)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))+...
    Tbt(2,3)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))+Tbt(3,3)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312))));

theta421=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),-(Tbt(1,3)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))+...
    Tbt(2,3)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))+Tbt(3,3)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321))));

theta422=atan2(Tbt(1,3)*sin(theta12)-Tbt(2,3)*cos(theta12),-(Tbt(1,3)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))+...
    Tbt(2,3)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))+Tbt(3,3)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322))));

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

i11=-sin(theta211)*sin(theta311)+cos(theta211)*cos(theta311);
i12=-sin(theta212)*sin(theta312)+cos(theta212)*cos(theta312);
i21=-sin(theta221)*sin(theta321)+cos(theta221)*cos(theta321);
i22=-sin(theta222)*sin(theta322)+cos(theta222)*cos(theta322);

theta511=atan2(-(Tbt(1,3)*a11+Tbt(2,3)*b11+Tbt(3,3)*c11),Tbt(1,3)*g11+Tbt(2,3)*h11+Tbt(3,3)*i11);
theta512=atan2(-(Tbt(1,3)*a12+Tbt(2,3)*b12+Tbt(3,3)*c12),Tbt(1,3)*g12+Tbt(2,3)*h12+Tbt(3,3)*i12);
theta521=atan2(-(Tbt(1,3)*a21+Tbt(2,3)*b21+Tbt(3,3)*c21),Tbt(1,3)*g21+Tbt(2,3)*h21+Tbt(3,3)*i21);
theta522=atan2(-(Tbt(1,3)*a22+Tbt(2,3)*b22+Tbt(3,3)*c22),Tbt(1,3)*g22+Tbt(2,3)*h22+Tbt(3,3)*i22);

if abs(theta511)<1e-10 || abs(theta512)<1e-10 || abs(theta521)<1e-10 || abs(theta522)<1e-10 ||...
        abs(theta511-pi)<1e-10 || abs(theta512-pi)<1e-10 || abs(theta521-pi)<1e-10 || abs(theta522-pi)<1e-10
    display('Singularity due to theta5 = k*pi, theta4 and theta6 are not independent and have multiple solutions.');
end

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

p11=sin(theta411)*(cos(theta11)*cos(theta211)*cos(theta311)-cos(theta11)*sin(theta211)*sin(theta311))+sin(theta11)*cos(theta411);
p12=sin(theta412)*(cos(theta11)*cos(theta212)*cos(theta312)-cos(theta11)*sin(theta212)*sin(theta312))+sin(theta11)*cos(theta412);
p21=sin(theta421)*(cos(theta12)*cos(theta221)*cos(theta321)-cos(theta12)*sin(theta221)*sin(theta321))+sin(theta12)*cos(theta421);
p22=sin(theta422)*(cos(theta12)*cos(theta222)*cos(theta322)-cos(theta12)*sin(theta222)*sin(theta322))+sin(theta12)*cos(theta422);

q11=sin(theta411)*(sin(theta11)*cos(theta211)*cos(theta311)-sin(theta11)*sin(theta211)*sin(theta311))-cos(theta11)*cos(theta411);
q12=sin(theta412)*(sin(theta11)*cos(theta212)*cos(theta312)-sin(theta11)*sin(theta212)*sin(theta312))-cos(theta11)*cos(theta412);
q21=sin(theta421)*(sin(theta12)*cos(theta221)*cos(theta321)-sin(theta12)*sin(theta221)*sin(theta321))-cos(theta12)*cos(theta421);
q22=sin(theta422)*(sin(theta12)*cos(theta222)*cos(theta322)-sin(theta12)*sin(theta222)*sin(theta322))-cos(theta12)*cos(theta422);

r11=sin(theta411)*(sin(theta211)*cos(theta311)+cos(theta211)*sin(theta311));
r12=sin(theta412)*(sin(theta212)*cos(theta312)+cos(theta212)*sin(theta312));
r21=sin(theta421)*(sin(theta221)*cos(theta321)+cos(theta221)*sin(theta321));
r22=sin(theta422)*(sin(theta222)*cos(theta322)+cos(theta222)*sin(theta322));

theta611=atan2(-(Tbt(1,1)*p11+Tbt(2,1)*q11+Tbt(3,1)*r11),Tbt(1,1)*j11+Tbt(2,1)*k11+Tbt(3,1)*l11);
theta612=atan2(-(Tbt(1,1)*p12+Tbt(2,1)*q12+Tbt(3,1)*r12),Tbt(1,1)*j12+Tbt(2,1)*k12+Tbt(3,1)*l12);
theta621=atan2(-(Tbt(1,1)*p21+Tbt(2,1)*q21+Tbt(3,1)*r21),Tbt(1,1)*j21+Tbt(2,1)*k21+Tbt(3,1)*l21);
theta622=atan2(-(Tbt(1,1)*p22+Tbt(2,1)*q22+Tbt(3,1)*r22),Tbt(1,1)*j22+Tbt(2,1)*k22+Tbt(3,1)*l22);

theta=[theta11 theta211 theta311 theta411 theta511 theta611;
       theta11 theta212 theta312 theta412 theta512 theta612;
       theta12 theta221 theta321 theta421 theta521 theta621;
       theta12 theta222 theta322 theta422 theta522 theta622;
       theta11 theta211 theta311 theta411-pi -theta511 theta611-pi;
       theta11 theta212 theta312 theta412-pi -theta512 theta612-pi;
       theta12 theta221 theta321 theta421-pi -theta521 theta621-pi;
       theta12 theta222 theta322 theta422-pi -theta522 theta622-pi];
for i=1:48
    while theta(i)<-pi
        if theta(i)<-pi
            theta(i)=theta(i)+2*pi;
        end
    end
    while theta(i)>pi
        if theta(i)>pi
            theta(i)=theta(i)-2*pi;
        end
    end
end
end

