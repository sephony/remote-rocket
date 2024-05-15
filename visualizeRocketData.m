% 功能: 可视化火箭主动段各参数数据
function visualizeRocketData(display, X_powered, t_powered)

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
% 截取一级数据长度
index_stage1 = find(t_powered == display.t_stage(1));
% 计算最大攻角、最大动压和最大法向过载
[max_alpha, idx_max_alpha] = min(alpha_display(1:index_stage1));
[max_q, idx_max_q] = max(q_display(1:index_stage1));
[max_n, idx_max_n] = max(n_display(1:index_stage1));

% 打印到终端
fprintf('一级飞行时最大攻角: %f° 在时间: %fs\n', max_alpha, t_powered(idx_max_alpha));
fprintf('一级飞行时最大动压: %fPa 在时间: %fs\n', max_q, t_powered(idx_max_q));
fprintf('一级飞行时最大法向过载: %fg 在时间: %fs\n', max_n, t_powered(idx_max_n));

% 绘制主动段数据
figure(4);
hold on;  % 允许在同一张图上绘制多条曲线

% 绘制攻角
plot(t_powered, alpha_display, 'r');  % 使用红色
plot(t_powered(idx_max_alpha), max_alpha, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_alpha), max_alpha, sprintf('一级飞行时最大攻角:\n%f°', max_alpha));
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
hold on;
plot(t_powered, q_display);
plot(t_powered(idx_max_q), max_q, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_q), max_q, sprintf('一级飞行时最大动压:\n%fPa', max_q));
xlabel('时间/s');
ylabel('动压/Pa');
title('动压随时间变化');
grid on;
hold off;

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
hold on;
plot(t_powered, n_display);
plot(t_powered(idx_max_n), max_n, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_n), max_n, sprintf('一级飞行时最大法向过载:\n%fg', max_n));
xlabel('时间/s');
ylabel('过载/g');
title('过载随时间变化');
grid on;
hold off;
end
