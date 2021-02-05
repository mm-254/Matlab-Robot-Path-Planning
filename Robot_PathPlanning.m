%Robot path planning for robot types RRR and RR
%Mandira Marambe

clear;
clc;
close all;

% Load file containing set points and time steps
info = "info_RRR.txt";
info =load(info, " ");

%Load robot_type file

%Parameters loaded from info file
total_time = info(1,1);
time_step = info(1,2);
s_acc = info(1,3);
s_dec = info(1,4);
 
P_i =[info(2,1), info(2,2), info(2,3)];
P_f =[info(3,1), info(3,2), info(3,3)];
steps = total_time/time_step;

%Maximum velocity
max_Pdot = (P_f-P_i)./((steps-0.5*s_dec-0.5*s_acc)*time_step);
A_star = max_Pdot.*time_step; 

%Generate Time vector and Position vectors through time
time_vec = zeros(steps+1,1);

dP =(P_f-P_i)./steps;
P_vec=zeros(steps+1,3);
P_vec(1,1) =P_i(1,1);
P_vec(1,2)=P_i(1,2);
P_vec(1,3)=P_i(1,3);
P_dot=zeros(steps+1,3);

for i = 1:steps+1
    %Time vector
    if i<steps+1
       time_vec(i+1,1) = time_vec(i)+time_step;
    end
    
    
    %Cartesian Positions and Velocities
    
    if i<s_acc+1
       
       P_dot(i+1,1) = P_dot(i,1)+ (max_Pdot(1,1)/(s_acc));
       P_dot(i+1,2) = P_dot(i,2)+(max_Pdot(1,2)/(s_acc));
       P_dot(i+1,3) = P_dot(i,3)+(max_Pdot(1,3)/(s_acc));
       
       %Px, Py, Pz
       P_vec(i+1,1) = P_vec(i,1)+0.5*(P_dot(i+1,1)+P_dot(i,1))*time_step;
       P_vec(i+1,2) = P_vec(i,2)+0.5*(P_dot(i+1,2)+P_dot(i,2))*time_step;
       P_vec(i+1,3) = P_vec(i,3)+0.5*(P_dot(i+1,2)+P_dot(i,2))*time_step;
       
    elseif i<steps-s_dec+1
       %Px, Py, Pz
       P_vec(i+1,1) = P_vec(i,1)+A_star(1,1);
       P_vec(i+1,2) = P_vec(i,2)+A_star(1,2);
       P_vec(i+1,3) = P_vec(i,3)+A_star(1,3);
       
       P_dot(i+1,1) = max_Pdot(1,1);
       P_dot(i+1,2) = max_Pdot(1,2);
       P_dot(i+1,3) = max_Pdot(1,3);
       
    elseif i <steps+1
       P_dot(i+1,1) = P_dot(i,1)- (max_Pdot(1,1)/(s_dec));
       P_dot(i+1,2) = P_dot(i,2)-(max_Pdot(1,2)/(s_dec));
       P_dot(i+1,3) = P_dot(i,3)-(max_Pdot(1,3)/(s_dec)); 
       
       %Px, Py, Pz
       P_vec(i+1,1) = P_vec(i,1)+0.5*(P_dot(i+1,1)+P_dot(i,1))*time_step;
       P_vec(i+1,2) = P_vec(i,2)+0.5*(P_dot(i+1,2)+P_dot(i,2))*time_step;
       P_vec(i+1,3) = P_vec(i,3)+0.5*(P_dot(i+1,3)+P_dot(i,3))*time_step;
       
    end
    
   
end

 %Generate joint angles and rates
 
 [theta, theta_dot] = robot_type("RRR", P_vec, P_dot);
 
%Generate Plots
%Position of end effector
figure(1)
hold on
%plot3(time_vec, P_vec(:,1),P_vec(:,2));
plot(time_vec, P_vec(:,1));
plot(time_vec, P_vec(:,2));
legend('Px','Py');
xlabel('Time');
ylabel('Cartesian Position');
%zlabel('Cartesian Position Y');
hold off

%Joint angles and rates
figure(2)
hold on
plot(time_vec, theta(:,1));
plot(time_vec, theta(:,2));
plot(time_vec, theta(:,3));
legend('Theta 1','Theta 2', 'Theta 3');
xlabel('Time');
ylabel('Joint angles');
hold off

figure(3)
hold on
plot(time_vec, theta_dot(:,1));
plot(time_vec, theta_dot(:,2));
plot(time_vec, theta_dot(:,3));
legend('W 1','W 2', 'W 3');
xlabel('Time');
ylabel('Joint rates');
hold off

Output = [time_vec,P_vec, theta, theta_dot];

%Output file
fileID = fopen('output_RRR.txt','w');
fprintf(fileID,'%6s %6.1f\n','m',steps+1);
fprintf(fileID,'%6s %6s %6s %6s %8s %8s %8s %8s %8s %8s\n','Time','Px','Py','Pz', 'Th1','Th2','Th3','w1','w2','w3');
fprintf(fileID,'%6.2f %6.3f %6.3f %6.3f %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f\n',transpose(Output));
fclose(fileID);


