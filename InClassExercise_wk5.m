function T05 = fanuc_fk(q)  % q = [q1 q2 q3 q4 q5] in rad
    % 如果沒有輸入參數，使用預設值並繪圖
    if nargin == 0
        q = deg2rad([45 -20 -30 0 60 180]);
        show_results = true;
    else
        show_results = false;
    end
    
    % DH參數
    a  = [-110 -770 100 0 0 0];
    d  = [525   0   0   -740 0 100];
    al = [pi/2 0 pi/2 pi/2 pi/2 0];
    th = [q(1)+pi, q(2)-pi/2, q(3)+pi, q(4)+0, q(5)+pi, q(6)];

    % 計算正向運動學
    T05 = eye(4);
    z = zeros(3,7); % z0~z6
    o = zeros(3,7); % o0~o6
    z(:,1) = [0;0;1];
    o(:,1) = [0;0;0];
    T_temp = eye(4);
    for i = 1:6
        ct = cos(th(i));  st = sin(th(i));
        ca = cos(al(i));  sa = sin(al(i));
        A = [ ct, -st*ca,  st*sa, a(i)*ct;
              st,  ct*ca, -ct*sa, a(i)*st;
               0,     sa,     ca,    d(i);
               0,      0,      0,      1 ];
        T05 = T05 * A;
        T_temp = T_temp * A;
        z(:,i+1) = T_temp(1:3,3);
        o(:,i+1) = T_temp(1:3,4);
    end
    % Analytical/Geometric Jacobian
    o6 = o(:,7);
    Jv = zeros(3,6);
    Jw = zeros(3,6);
    for i = 1:6
        Jv(:,i) = cross(z(:,i), o6 - o(:,i));
        Jw(:,i) = z(:,i);
    end
    J = [Jv; Jw];

    % 顯示結果和繪圖
    if show_results
        fprintf('末端位置: [%.1f, %.1f, %.1f] mm\n', T05(1:3,4));
        fprintf('關節角度: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] 度\n', rad2deg(q));
        fprintf('Analytical/Geometric Jacobian (6x6):\n');
        disp(J);
        plot_robot(q, a, d, al, th);
    end
end

function plot_robot(q, a, d, al, th)
    % 計算關節位置
    T = eye(4);
    pos = zeros(6, 3);
    pos(1, :) = [0, 0, 0];  % 基座
    
    for i = 1:6
        ct = cos(th(i));  st = sin(th(i));
        ca = cos(al(i));  sa = sin(al(i));
        A = [ ct, -st*ca,  st*sa, a(i)*ct;
              st,  ct*ca, -ct*sa, a(i)*st;
               0,     sa,     ca,    d(i);
               0,      0,      0,      1 ];
        T = T * A;
        pos(i+1, :) = T(1:3, 4)';
    end
    
    % 繪圖
    figure; hold on; grid on; axis equal;
    
    % 繪製連桿和關節
    plot3(pos(:,1), pos(:,2), pos(:,3), 'b-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % 座標軸
    quiver3(0, 0, 0, 200, 0, 0, 'r', 'LineWidth', 2);
    quiver3(0, 0, 0, 0, 200, 0, 'g', 'LineWidth', 2);
    quiver3(0, 0, 0, 0, 0, 200, 'b', 'LineWidth', 2);
    
    % 末端座標軸
    end_x = T(1:3, 1) * 150; end_y = T(1:3, 2) * 150; end_z = T(1:3, 3) * 150;
    quiver3(pos(6,1), pos(6,2), pos(6,3), end_x(1), end_x(2), end_x(3), 'r', 'LineWidth', 2);
    quiver3(pos(6,1), pos(6,2), pos(6,3), end_y(1), end_y(2), end_y(3), 'g', 'LineWidth', 2);
    quiver3(pos(6,1), pos(6,2), pos(6,3), end_z(1), end_z(2), end_z(3), 'b', 'LineWidth', 2);
    
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title('FANUC Robot'); view(45, 30);
    
    % 顯示角度資訊
    text(min(pos(:,1))-200, min(pos(:,2))-200, max(pos(:,3))+200, ...
         sprintf('q = [%.0f°, %.0f°, %.0f°, %.0f°, %.0f°, %.0f°]', rad2deg(q)), ...
         'FontSize', 10, 'BackgroundColor', 'white');
end 

