function[joint_angles, joint_rates]= robot_type(type, positions, velocities)

joint_angles = zeros(length(positions),3);
joint_rates = zeros(length(positions),3);

if type == "RRR"
    
    L1=5; L2=5; L3=4; d1=4;
    
    for i=1: length(positions)
        
        a = sqrt((positions(i,1))^2 + (positions(i,2))^2);

        joint_angles(i,1) = atan2(positions(i,2),positions(i,1))-acos(d1/a);
        theta1 = joint_angles(i,1);
        
        b = (positions(i,1))-d1*cos(theta1);
        c = (positions(i,2))-d1*sin(theta1);
        d = (positions(i,3))-L1;
        e = (positions(i,1))*sin(theta1) -(positions(i,2))*cos(theta1);

        f = (b^2 + c^2 + d^2- L2^2 - L3^2)/(2*L2*L3);
        
        joint_angles(i,3)=acos(f);
        theta3=joint_angles(i,3);
        
        num=e*(L2+L3*cos(theta3))-(d*L3*sin(theta3));
        denom=d*(L2+L3*cos(theta3))+(e*L3*sin(theta3));

        joint_angles(i,2) = atan2(num,denom);
        theta2 = joint_angles(i,2);

        Jacobian = [-positions(i,2),L3*sin(theta1)*cos(theta2+theta3) + L2*sin(theta1)*cos(theta2),L3*sin(theta1)*cos(theta2+theta3);
            positions(i,1),-L3*cos(theta1)*cos(theta2+theta3) - L2*cos(theta1)*cos(theta2),-L3*cos(theta1)*cos(theta2+theta3);
            0, -L3*sin(theta2+theta3) - L2*sin(theta2),-L3*sin(theta2+theta3)];

        joint_rates(i,:) = (inv(Jacobian)) * transpose(velocities(i,:));
        
        joint_angles(i,1) =rad2deg(joint_angles(i,1));
        joint_angles(i,2)=rad2deg(joint_angles(i,2));
        joint_angles(i,3)=rad2deg(joint_angles(i,3));
        
    end
    
elseif type =="RR"
    
    L1=20; L2=20; 
    
    for i= 1:length(positions)
        a = ((positions(i,1))^2 + (positions(i,2))^2 + L1^2 - L2^2)/(2*L1);
        b = sqrt((positions(i,1))^2 + (positions(i,2))^2);

        joint_angles(i,1) = atan2(positions(i,2),positions(i,1))+acos(a/b);
        theta1 = joint_angles(i,1);
        
        c=positions(i,2)-L1*sin(theta1);
        d=positions(i,1)-L1*cos(theta1);
        
        joint_angles(i,2) = atan2(c,d)-theta1;
        theta2 = joint_angles(i,2);

        Jacobian = [- L2*sin(theta1 + theta2) - L1*sin(theta1), - L2*sin(theta1 + theta2);
        L2*cos(theta1 + theta2) + L1*cos(theta1), L2*cos(theta1 + theta2)];

        joint_rates(i,1:2) = (inv(Jacobian)) * transpose(velocities(i,1:2));
        
        joint_angles(i,1) =rad2deg(joint_angles(i,1));
        joint_angles(i,2)=rad2deg(joint_angles(i,2));
    end
end

