function [] = check(teta,theta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
aux=theta';

direct_kinematics(teta)
for i=1:8
   direct_kinematics(aux(1+6*(i-1):6*i)) 
end
end

