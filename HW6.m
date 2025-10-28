clc; clear; close all;
x = 33.23; y=64.54; z=232.32;

% DH table
a = [11.23,150,100];
alpha = [pi/2,0,0];
d = [92.35 , 0 , 0];

%% derive inverse kinematic
% cosine law c^2 = a^2+b^2-2abcos(gamma)
theta1 = atan2(y, x);
z1 = z - d(1);
k1 = sqrt(x^2 + y^2) - a(1);
r = sqrt(k1^2 + z1^2);
d1 = a(2);
d2 = a(3);

theta3 = acos((r^2- (d1^2)-(d2^2)) / (2 * d1 * d2));

beta1 = atan2(z1, k1);
beta2 = atan(d2*sin(theta3)/(d1+d2*cos(theta3)));
theta2 = beta1 - beta2;
% 印出逆運動學結果
disp(['theta1 = ', num2str(theta1)]);
disp(['theta2 = ', num2str(theta2)]);
disp(['theta3 = ', num2str(theta3)]);
% 轉換為角度並輸出
disp(['theta1 (deg) = ', num2str(rad2deg(theta1))]);
disp(['theta2 (deg) = ', num2str(rad2deg(theta2))]);
disp(['theta3 (deg) = ', num2str(rad2deg(theta3))]);


% 正向運動學計算每個關節位置
T01 = trotz(theta1) * transl([0 0 d(1)]) * transl([a(1) 0 0]) * trotx(alpha(1));
T12 = trotz(theta2) * transl([0 0 d(2)]) * transl([a(2) 0 0]) * trotx(alpha(2));
T23 = trotz(theta3) * transl([0 0 d(3)]) * transl([a(3) 0 0]) * trotx(alpha(3));

T02 = T01 * T12;
T03 = T02 * T23;

% 各關節座標
p0 = [0 0 0]';
p1 = T01(1:3,4);
p2 = T02(1:3,4);
p3 = T03(1:3,4);

% 繪圖
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