function visualizeRocketData(display, X_powered, t_powered, X_whole)
%% 绘制主动段弹道曲线（发射坐标系下）
% 获取各级关机点时间对应的索引
idx_stage1 = find(t_powered == display.t_stage(1), 1);
idx_stage2 = find(t_powered == display.t_stage(1) + display.t_stage(2), 1);
idx_stage3 = find(t_powered == display.t_stage(1) + display.t_stage(2) + display.t_stage(3), 1);
vec_idx = [idx_stage1, idx_stage2, idx_stage3];

figure (1);
hold on
plot3(X_powered(:,1),X_powered(:,3),X_powered(:,2));
plotBornoutPoint3(X_powered, vec_idx);
hold off
view(3);
axis equal;
grid on;
xlabel('x/m');
ylabel('z/m');
zlabel('y/m');
title('发射坐标系下主动段弹道曲线');

%% 绘制全弹道曲线（发射坐标系下）
figure (2);
hold on
plot3(X_whole(:,1),X_whole(:,3),X_whole(:,2));
plotBornoutPoint3(X_whole, vec_idx);
hold off
view(3);
axis equal;
grid on;
xlabel('x/m');
ylabel('z/m');
zlabel('y/m');
title('发射坐标系下全弹道曲线');

R = X_whole(:,1:3);
% 所有时刻的地心坐标系下火箭地心矢量
R_E = Rotation.L2E(display.A_L0, display.theta_T0, display.Phi_T0) * R' + display.R0_e;
R_E = R_E';
%% 在地球上可视化弹道曲线（地心坐标系下）
figure(3);
hold on
traj = plot3(R_E(:,1), R_E(:,3), R_E(:,2));
traj.LineWidth = 3;
view(3);
ellipsoid(0, 0, 0, Earth.a_e, Earth.a_e, Earth.b_e);
plotBornoutPoint3(R_E, vec_idx);
hold off
axis equal
grid on
title('地心坐标系下弹道曲线');

%% 可视化火箭主动段各参数数据
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
    h_display(i) = display.h * 0.001;
    v_display(i) = display.v;
    m_display(i) = display.m;
    q_display(i) = display.q * 0.001;
    theta_L_display(i) = Earth.rad2deg(display.theta_L);
    Phi_L_display(i) =  Earth.rad2deg(display.Phi_L);
    n_display(i) = display.n;
end

%% 绘制一级飞行时最大攻角、最大动压和最大法向过载
% 计算一级飞行最大攻角、最大动压和最大法向过载
[max_alpha, idx_max_alpha] = min(alpha_display(1:idx_stage1));
[max_q, idx_max_q] = max(q_display(1:idx_stage1));
[max_n, idx_max_n] = max(n_display(1:idx_stage1));

% 打印到终端
fprintf('一级飞行时最大攻角: %f° 在时间: %fs\n', max_alpha, t_powered(idx_max_alpha));
fprintf('一级飞行时最大动压: %fPa 在时间: %fs\n', max_q, t_powered(idx_max_q));
fprintf('一级飞行时最大法向过载: %fg 在时间: %fs\n', max_n, t_powered(idx_max_n));

figure(4);
hold on  % 允许在同一张图上绘制多条曲线
% 绘制攻角
plot(t_powered, alpha_display, 'r');  % 使用红色
plot(t_powered(idx_max_alpha), max_alpha, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_alpha), max_alpha, sprintf('一级飞行时最大攻角:\n%.2f°', max_alpha));
% 绘制俯仰角
plot(t_powered, pitch_display, 'g');  % 使用绿色
% 绘制弹道倾角
plot(t_powered, theta_v_display, 'b');  % 使用蓝色
hold off  % 结束绘制多条曲线

xlabel('时间/s');
ylabel('角度/°');
title('攻角、俯仰角和弹道倾角随时间变化');
legend('攻角', '俯仰角', '弹道倾角');  % 添加图例
ylim([-40 90]);  % 设置y轴范围为0-90°
grid on;

%% 绘制主动段数据
figure(5);
subplot(3,2,1);
hold on
plot(t_powered, h_display);
plotBornoutPoint(t_powered, h_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('高度/km');
title('高度随时间变化');
grid on;

subplot(3,2,2);
hold on
plot(t_powered, v_display);
plotBornoutPoint(t_powered, v_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('速度/m/s');
title('速度随时间变化');
grid on;

subplot(3,2,3);
hold on
plot(t_powered, q_display);
plot(t_powered(idx_max_q), max_q, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_q), max_q, sprintf('一级飞行时最大动压:\n%.2fkPa', max_q));
plotBornoutPoint(t_powered, q_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('动压/Pa');
title('动压随时间变化');
grid on;

subplot(3,2,4);
hold on
plot(t_powered, m_display);
plotBornoutPoint(t_powered, m_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('质量/kg');
title('质量随时间变化');
grid on;

subplot(3,2,5);
hold on
plot(t_powered, theta_L_display);
plotBornoutPoint(t_powered, theta_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理经度/°');
title('地理经度随时间变化');
grid on;

subplot(3,2,6);
hold on
plot(t_powered, Phi_L_display);
plotBornoutPoint(t_powered, Phi_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理纬度/°');
title('地理纬度随时间变化');
grid on;

figure(6);
hold on
plot(t_powered, n_display);
plot(t_powered(idx_max_n), max_n, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_n), max_n, sprintf('一级飞行时最大法向过载:\n%.2fg', max_n));
plotBornoutPoint(t_powered, n_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('过载/g');
title('过载随时间变化');
grid on;
end

function plotBornoutPoint(t, data, indexs)
for i = 1:length(indexs)
    plot(t(indexs(i)), data(indexs(i)), '*');
end
end

function plotBornoutPoint3(X, indexs)
for i = 1:length(indexs)
    plot3(X(indexs(i),1), X(indexs(i),3), X(indexs(i),2), '*');
end
end
