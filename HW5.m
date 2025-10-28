clc; clear; close all;

%% ------------------- DH PARAMETERS -------------------
a = [13.6, 150, 100];        
alpha = [pi/2, 0, 0];        
d = [93.175, 0, 0];          

%% ------------------- INITIAL Guess -------------------
theta = [0; 0; 0];           

%% ------------------- TARGET END-EFFECTOR POSITION -------------------
target_pos = [50; 20; 250];

%% ------------------- ITERATION PARAMETERS -------------------
dt = 0.1;       
k = 0.5;        
max_iter = 100;
tol = 1e-6;     

%% ------------------- RECORD VARIABLES -------------------
history_error = zeros(1, max_iter);

%% ------------------- DIFFERENTIAL IK -------------------
for iter = 1:max_iter
    % FK for ee position (Forward Kinematics is given)
    T = eye(4);
    for i = 1:3
        A = [cos(theta(i)) -sin(theta(i))*cos(alpha(i))  sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
             sin(theta(i))  cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
             0              sin(alpha(i))                cos(alpha(i))               d(i);
             0              0                            0                           1];
        T = T * A;
    end


    % calculate the position error
    current_pos = T(1:3,4);

    % ----- 【exercise 1】-----
    % Hint: calculate the difference between target and current position
    delta_x = target_pos - current_pos;

    history_error(iter) = norm(delta_x);
    if history_error(iter) < tol, break; end

    % Jacobian (Jacobian calculation is given)
    J = zeros(3,3);
    T_temp = eye(4);
    for i = 1:3
        if i == 1
            z = [0;0;1]; p = [0;0;0];
        else
            p = T_temp(1:3,4); z = T_temp(1:3,3);
        end
        J(:,i) = cross(z, current_pos - p);
        A = [cos(theta(i)) -sin(theta(i))*cos(alpha(i))  sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
             sin(theta(i))  cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
             0              sin(alpha(i))                cos(alpha(i))               d(i);
             0              0                            0                           1];
        T_temp = T_temp * A;
    end

    % ----- 【exercise 2】-----
    % Hint1: dx = J x dtheta, find the dtheta
    % Hint2: suggest pinv() to find inverse matrix
    theta_dot = pinv(J) * (k * delta_x);

    % ----- 【exercise 3】-----
    % 提示：euler integrating to find new theta
    theta = theta + theta_dot * dt;

    % 紀錄每次的 T 以便繪圖
    if iter == 1
        T_list = zeros(4,4,3);
    end
    T_temp = eye(4);
    for i = 1:3
        A = [cos(theta(i)) -sin(theta(i))*cos(alpha(i))  sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
             sin(theta(i))  cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
             0              sin(alpha(i))                cos(alpha(i))               d(i);
             0              0                            0                           1];
        T_temp = T_temp * A;
        T_list(:,:,i) = T_temp;
    end
end

%% ------------------- RESULT -------------------
disp('Final joint angles (rad):');
disp(theta);
disp('Final end-effector position:');
disp(current_pos);
euc_dist = norm(target_pos - current_pos);
disp(['Euclidean distance to target: ', num2str(euc_dist)]);

%% ------------------- PLOT CONVERGENCE ERROR -------------------
figure;
plot(1:length(history_error), history_error, '-', 'LineWidth', 1);
yline(tol,'--','Color','r'); 
xlabel('Iteration'); ylabel('End-Effector Position Error (Euclidean)');
title('Convergence of Differential IK');
grid on;

%% ------------------- PLOT FINAL ARM POSTURE -------------------
figure; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Final Arm Posture');
view(40,20);

base = [0;0;0];
prev_pos = base;

% use for-loop to plot joint, after drawing joint i become prev joint
for i = 1:3
    joint_pos = T_list(1:3,4,i);
    plot3([prev_pos(1), joint_pos(1)], [prev_pos(2), joint_pos(2)], [prev_pos(3), joint_pos(3)], '-o','LineWidth',2,'MarkerSize',6);
    prev_pos = joint_pos;
end
plot3(target_pos(1), target_pos(2), target_pos(3), 'rx','MarkerSize',10,'LineWidth',2);
legend('Links','Target');
