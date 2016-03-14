function [ teta ] = pro2( pose )
%   pro2 : Computes angles of the hand of a 6-dof arm-robot (should do it...)
%   Receives variable pose of dimensions [1 6] and using moving axes
%   rotation matrixes, returns a matrix with the various combinations of angles
Rzyx=[cos(pose(4))*cos(pose(5)) cos(pose(4))*sin(pose(5))*sin(pose(6))-sin(pose(4))*cos(pose(6))  cos(pose(4))*sin(pose(5))*cos(pose(6))+sin(pose(4))*sin(pose(6));
sin(pose(4))*cos(pose(5)) sin(pose(4))*sin(pose(5))*sin(pose(6))+cos(pose(4))*cos(pose(6)) sin(pose(4))*sin(pose(5))*cos(pose(6))-cos(pose(4))*sin(pose(6)); 
-sin(pose(5)) cos(pose(5))*sin(pose(6)) cos(pose(5))*cos(pose(6))];
P=[pose(1) pose(2) pose(3)]';
Tbt=[Rzyx P;0 0 0 1]

end

