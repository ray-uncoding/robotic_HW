%% InClassExercise_wk7 - 基於 hw6

% 此題情境為三軸機械手臂，DH 參數如下：
% a = [11.23, 150, 100];
% alpha = [pi/2, 0, 0];
% d = [92.35, 0, 0];
% theta = [0, 0, 0];  % 注意：theta 在此為未知量，需要計算出來

%% HW6 要求
% 1. 計算逆運動學，求出各關節角度 theta1, theta2, theta3，使末端到達指定位置 (x, y, z)
% 2. 計算正向運動學，取得各關節位置

%% HW7 要求
% 1. 基於 HW6 的逆運動學與正向運動學結果，推導並計算 Analytical Jacobian 矩陣 J
% 2. 使用微分逆運動學方法，求出對應於 Week 6 分析的配置的關節向量。
% 3. 比較微分逆運動學結果與 Week 6 逆運動學結果，並討論其異同。

%% Main Script

clc; clear; close all;

%% === STEP 0 參數設定 ===

% 目標末端位置 [x, y, z]
x = 33.23; y = 64.54; z = 232.32;

% DH 參數 [ a, alpha, d, theta ]
% 注意：theta 在此為未知量，待會會計算出來
a       = [11.23, 150, 100];
alpha   = [pi/2, 0, 0];
d       = [92.35, 0, 0];

%% === STEP 1 逆運動學計算 ===
theta1 = atan2(y, x);
z1 = z - d(1);
k1 = sqrt(x^2 + y^2) - a(1);
r = sqrt(k1^2 + z1^2);
d1 = a(2);
d2 = a(3);
theta3 = acos((r^2 - d1^2 - d2^2) / (2 * d1 * d2));
beta1 = atan2(z1, k1);
beta2 = atan(d2 * sin(theta3) / (d1 + d2 * cos(theta3)));
theta2 = beta1 - beta2;
disp('--- Analytical Inverse Kinematics ---');
disp(['theta1 = ', num2str(theta1)]);
disp(['theta2 = ', num2str(theta2)]);
disp(['theta3 = ', num2str(theta3)]);
disp(['theta1 (deg) = ', num2str(rad2deg(theta1))]);
disp(['theta2 (deg) = ', num2str(rad2deg(theta2))]);
disp(['theta3 (deg) = ', num2str(rad2deg(theta3))]);

% === STEP 2 正向運動學計算 (取得各關節位置) ===
T01 = trotz(theta1) * transl([0 0 d(1)]) * transl([a(1) 0 0]) * trotx(alpha(1));
T12 = trotz(theta2) * transl([0 0 d(2)]) * transl([a(2) 0 0]) * trotx(alpha(2));
T23 = trotz(theta3) * transl([0 0 d(3)]) * transl([a(3) 0 0]) * trotx(alpha(3));
T02 = T01 * T12;
T03 = T02 * T23;

% === STEP 3 Analytical Jacobian ===
z0 = [0; 0; 1];
o0 = [0; 0; 0];
o1 = T01(1:3, 4);
z1 = T01(1:3, 3);
o2 = T02(1:3, 4);
z2 = T02(1:3, 3);
o3 = T03(1:3, 4); % 末端點
J1 = [cross(z0, o3 - o0); z0];
J2 = [cross(z1, o3 - o1); z1];
J3 = [cross(z2, o3 - o2); z2];
J = [J1 J2 J3];
Jv = J(1:3, :);
disp('--- Analytical Jacobian (linear part) ---');
disp(Jv);

%% === STEP 4 微分逆運動學與比較 ===

% 計算末端執行器的速度 (只用線速度)

v = [100; 300; 0]; % 末端執行器線速度
Jv_inv = pinv(Jv); % 取 Jv 的廣義逆
q_dot = Jv_inv * v; % 計算關節速度

disp('--- Differential Inverse Kinematics ---');
disp(['q_dot = ', num2str(q_dot')]);

% === STEP 99 機械手臂姿態繪圖 ===
p0 = [0 0 0]';
p1 = T01(1:3, 4);
p2 = T02(1:3, 4);
p3 = T03(1:3, 4);
figure;
plot3([p0(1) p1(1) p2(1) p3(1)], [p0(2) p1(2) p2(2) p3(2)], [p0(3) p1(3) p2(3) p3(3)], '-o', 'LineWidth', 2);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('三軸機械手臂姿態');


% --- 輔助函數區塊 ---
function T = trotz(theta)
    T = [cos(theta) -sin(theta) 0 0;
         sin(theta)  cos(theta) 0 0;
         0           0          1 0;
         0           0          0 1];
end

function T = trotx(alpha)
    T = [1 0           0          0;
         0 cos(alpha) -sin(alpha) 0;
         0 sin(alpha)  cos(alpha) 0;
         0 0           0          1];
end

function T = transl(v)
    T = eye(4);
    T(1:3,4) = v(:);
end