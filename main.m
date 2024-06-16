clc; clear; close all;
tic;
%% 火箭发射点参数
A_L0 = deg2rad(-20);      % 发射点地理方位角
theta_L0 = deg2rad(60);   %（东经为正，西经为负）
Phi_L0 = deg2rad(-30);    %（北纬为正，南纬为负）

pitch_data = load('data/FiC.txt');   % 读取俯仰角飞行程序数据
rocket = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);    % 创建火箭对象
disp('发射点参数:')
fprintf('发射方位角: %.2f°  地理经度: %.2f°  地理纬度: %.2f°\n\n', rad2deg(A_L0), rad2deg(theta_L0), rad2deg(Phi_L0));

%% 微分方程参数设置
step = 1;                       % 定义外部循环步长,默认是 1 秒
N = floor(500000 / step);       % 预分配内存
X_count = zeros(N,7);           % N 行 7 列的矩阵, 7 列分别是 x, y, z, Vx, Vy, Vz, m
t_count = zeros(N,1);           % N 行 1 列的矩阵, 存储时间
index = 1;                      % 索引变量

%% 主动段弹道计算
tStart_powerd = tic;
for t = 0: step: (rocket.t_stage(2)+rocket.t_stage(1)+rocket.t_stage(3) - step)
    [t_t, X_t] = ode45(@dynamic, [t; t+step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
end
tEnd_powerd = toc(tStart_powerd);
fprintf('主动段解算的时间是 %.2f 秒\n', tEnd_powerd);

% 截取主动段数据
X_powered = X_count(1:index-1, :);
t_powered = t_count(1:index-1);

%% 被动段弹道计算
tStart_passive = tic;
for t = (rocket.t_stage(1) + rocket.t_stage(2) + rocket.t_stage(3)) : step : 100000
    [t_t, X_t] = ode45(@dynamic, [t, t + step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
    
    % if rocket.r < 6400000 % 粗略的落地判定，高度小于 6400km
    % 按步长精确的落地判定，高度比速度应小于两倍步长
    if (rocket.h > 0) && ((rocket.h / rocket.v) < (2 * step))
        fprintf('导弹打击时间为：%.2fs （相对误差 < %.2fs)\n',t, step);
        fprintf('导弹打击点经度：%.2f°, 纬度：%.2f°\n\n', rad2deg(rocket.theta_L), rad2deg(rocket.Phi_L));
        break;
    end
end
tEnd_passive = toc(tStart_passive);
fprintf('被动段解算的时间是 %.2f 秒\n', tEnd_passive);

% 截取全弹道数据
X_whole = X_count(1:index-1, :);
t_whole = t_count(1:index-1);
% 释放内存
clear X_count t_count;

%% 数据可视化
tStart_visualize = tic;
display = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);
visualizeRocketData(display, X_powered, t_powered, X_whole, t_whole);
tEnd_visualize = toc(tStart_visualize);
fprintf('数据可视化用的时间是 %.2f 秒\n\n', tEnd_visualize);
toc;
