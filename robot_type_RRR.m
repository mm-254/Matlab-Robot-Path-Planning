function[joint_angles, joint_rates]= robot_type_Planar_RRR(L1, L2, L3, positions, velocities)

joint_angles = zeros(length(positions),3);
joint_rates = zeros(length(positions),3);

for i= length(positions)
    joint_angles(i,1) = atan2(positions(i,1),positions(i,2));
    theta1 = joint_angles(i,1);
    joint_angles(i,2) = atan2(positions(i,1),positions(i,2));
    theta2 = joint_angles(i,2);
    joint_angles(i,3) = atan2(positions(i,1),positions(i,2));
    theta3 = joint_angles(i,3);
    
    Jacobian = [- L2*sind(theta1 + theta2) - L1*sind(theta1) - L3*sind(theta1 + theta2 + theta3), - L2*sind(theta1 + theta2) - L3*sind(theta1 + theta2 + theta3), -L3*sind(theta1 + theta2 + theta3);
    L2*cosd(theta1 + theta2) + L1*cosd(theta1) + L3*cosd(theta1 + theta2 + theta3), L2*cosd(theta1 + theta2) + L3*cosd(theta1 + theta2 + theta3), L3*cosd(theta1 + theta2 + theta3); 0,0,0];

    joint_rates = (inv(Jacobian)) * transpose(velocities(i,:));
end



