clc; clear;
%% 火箭发射点参数
A_L0 = -20 * pi / 180;
theta_L0 = 60 * pi / 180;
Phi_L0 = -30 * pi / 180;

pitch_data = load('FiC.txt');           % 读取俯仰角飞行程序数据
rocket = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);% 创建火箭对象

%% 微分方程参数设置
step = 1;               % 定义外部循环步长,默认是 1 秒
N = 500000 / step;      % 预分配内存
X_count = zeros(N,7);   % N 行 7 列的矩阵, 7 列分别是 x, y, z, Vx, Vy, Vz, m
t_count = zeros(N,1);   % N 行 1 列的矩阵, 存储时间
index = 1;              % 索引变量

%% 主动段弹道计算
% 一级
for t = 0: step: (rocket.t_stage(1) - step)
    [t_t, X_t] = ode45(@dynamic, [t; t+step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
end
% 二级

for t = rocket.t_stage(1): step: (rocket.t_stage(2)+rocket.t_stage(1) - step)
    [t_t, X_t] = ode45(@dynamic, [t; t+step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
end
% 三级

for t = (rocket.t_stage(2)+rocket.t_stage(1)): step: (rocket.t_stage(2)+rocket.t_stage(1)+rocket.t_stage(3) - step)
    [t_t, X_t] = ode45(@dynamic, [t; t + step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
end

% 截取主动段数据
X_powered = X_count(1:index-1, :);
t_powered = t_count(1:index-1);

% 绘制主动段弹道曲线（发射坐标系下）
figure (1);
plot3(X_powered(:,1),X_powered(:,3),X_powered(:,2));
axis equal;
grid on;
xlabel('x/m');
ylabel('z/m');
zlabel('y/m');
title('发射坐标系下主动段弹道曲线');

%% 被动段弹道计算
for t = (rocket.t_stage(1) + rocket.t_stage(2) + rocket.t_stage(3)) : step : 10000
    [t_t, X_t] = ode45(@dynamic, [t, t + step], rocket.X, [], rocket);
    
    num_rows = size(X_t, 1); % 计算 X_t 的行数
    X_count(index:(index+num_rows-1), :) = X_t; % 将 X_t 的数据插入到 X_count 中
    t_count(index:(index+num_rows-1)) = t_t; % 将 t_t 的数据插入到 t_count 中
    index = index + num_rows; % 更新索引变量
    
    rocket = rocket.update(t+1, X_count(index-1, :));
    
    % if rocket.r < 6400000 % 粗略的落地判定
    
    % 按步长精确的落地判定，高度比速度应小于两倍步长
    if (rocket.h > 0) && ((rocket.h / rocket.v) < (2 * step))
        fprintf('导弹打击时间为：%.2fs （误差 < %.2fs)\n',t, step);
        break;
    end
end

% 截取全弹道数据
X_whole = X_count(1:index-1, :);
t_whole = t_count(1:index-1);

% 绘制全弹道曲线（发射坐标系下）
figure (2);
plot3(X_whole(:,1),X_whole(:,3),X_whole(:,2));
axis equal;
grid on;
xlabel('x/m');
ylabel('z/m');
zlabel('y/m');
title('发射坐标系下全弹道曲线');

R = X_whole(:,1:3);
V = X_whole(:,4:6);
% 所有时刻的地心坐标系下火箭地心矢量
R_E = Rotation.L2E(rocket.A_L0, rocket.theta_T0, rocket.Phi_T0) * R' + rocket.R0_e;

%% 在地球上可视化弹道曲线（地心坐标系下）
figure(3);
traj = plot3(R_E(1,:), R_E(2,:), R_E(3,:));
traj.LineWidth = 3;
hold on
ellipsoid(0, 0, 0, Earth.a_e, Earth.a_e, Earth.b_e);
axis equal
grid on
title('地心坐标系下弹道曲线');

hold off

phi = Earth.rad2deg(rocket.Phi_L);
psi = Earth.rad2deg(rocket.theta_L);
if rocket.Rc_e(2) < 0
    psi = - psi;
end
fprintf('导弹打击点经度：%.2f°, 纬度：%.2f°\n', psi, phi);

display = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);
% 计算主动段数据长度
N_powered = size(t_powered,1);
h_display = zeros(N_powered ,1);
v_display = zeros(N_powered ,1);
theta_v_display = zeros(N_powered ,1);
q_display = zeros(N_powered ,1);
n_display = zeros(N_powered ,1);
alpha_display = zeros(N_powered ,1);
pitch_display = zeros(N_powered ,1);
m_display = zeros(N_powered ,1);
theta_L_display = zeros(N_powered ,1);
Phi_L_display = zeros(N_powered ,1);

% 将主动段数据赋值给 display
for i = 1:size(t_powered,1)
    display = display.update(t_powered(i), X_powered(i,:));
    pitch_display(i) = Earth.rad2deg(display.pitch);
    theta_v_display(i) = Earth.rad2deg(display.theta_v);
    alpha_display(i) = Earth.rad2deg(display.alpha);
    h_display(i) = display.h;
    v_display(i) = display.v;
    m_display(i) = display.m;
    q_display(i) = display.q;
    theta_L_display(i) = Earth.rad2deg(display.theta_L);
    Phi_L_display(i) =  Earth.rad2deg(display.Phi_L);
    n_display(i) = display.n;
end

%% 绘制主动段数据
figure(4);
hold on;  % 允许在同一张图上绘制多条曲线

% 绘制攻角
plot(t_powered, alpha_display, 'r');  % 使用红色
% 绘制俯仰角
plot(t_powered, pitch_display, 'g');  % 使用绿色
% 绘制弹道倾角
plot(t_powered, theta_v_display, 'b');  % 使用蓝色

xlabel('时间/s');
ylabel('角度/°');
title('攻角、俯仰角和弹道倾角随时间变化');
legend('攻角', '俯仰角', '弹道倾角');  % 添加图例

ylim([-40 90]);  % 设置y轴范围为0-90°

grid on;
hold off;  % 结束绘制多条曲线

figure(5);
subplot(3,2,1);
plot(t_powered, h_display);
xlabel('时间/s');
ylabel('高度/m');
title('高度随时间变化');
grid on;

subplot(3,2,2);
plot(t_powered, v_display);
xlabel('时间/s');
ylabel('速度/m/s');
title('速度随时间变化');
grid on;

subplot(3,2,3);
plot(t_powered, q_display);
xlabel('时间/s');
ylabel('动压/Pa');
title('动压随时间变化');
grid on;

subplot(3,2,4);
plot(t_powered, m_display);
xlabel('时间/s');
ylabel('质量/kg');
title('质量随时间变化');
grid on;

subplot(3,2,5);
plot(t_powered, theta_L_display);
xlabel('时间/s');
ylabel('地理经度/°');
title('地理经度随时间变化');
grid on;

subplot(3,2,6);
plot(t_powered, Phi_L_display);
xlabel('时间/s');
ylabel('地理纬度/°');
title('地理纬度随时间变化');
grid on;

figure(6);
plot(t_powered, n_display);
xlabel('时间/s');
ylabel('过载/g');
title('过载随时间变化');
grid on;





