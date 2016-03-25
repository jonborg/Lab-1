teta=unifrnd(-pi,pi,1,6);
theta=inverse_kinematics(direct_kinematics(teta));
check(teta,theta);