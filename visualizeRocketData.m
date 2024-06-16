function visualizeRocketData(display, X_powered, t_powered, X_whole, t_whole)
% 获取各级关机点时间对应的索引
idx_stage1 = find(t_powered == display.t_stage(1), 1);
idx_stage2 = find(t_powered == display.t_stage(1) + display.t_stage(2), 1);
idx_stage3 = find(t_powered == display.t_stage(1) + display.t_stage(2) + display.t_stage(3), 1);
vec_idx = [idx_stage1, idx_stage2, idx_stage3];

%% 绘制主动段弹道曲线（发射坐标系下）
figure (1);
hold on
plot3(X_powered(:,3),X_powered(:,1),X_powered(:,2));
plotShutdownPoint3(X_powered, vec_idx);
hold off
view(3);
axis equal;
grid on;
xlabel('z/m');
ylabel('x/m');
zlabel('y/m');
title('发射坐标系下主动段弹道曲线');
legend('主动段弹道曲线', '一级关机点', '二级关机点', '三级关机点');

%% 绘制全弹道曲线（发射坐标系下）
figure (2);
hold on
plot3(X_whole(:,3),X_whole(:,1),X_whole(:,2));
plotShutdownPoint3(X_whole, vec_idx);
hold off
view(3);
axis equal;
grid on;
xlabel('z/m');
ylabel('x/m');
zlabel('y/m');
title('发射坐标系下全弹道曲线');
legend('全弹道曲线', '一级关机点', '二级关机点', '三级关机点');

%% 在地球上可视化弹道曲线（地心坐标系下）
R = X_whole(:,1:3);
% 所有时刻的地心坐标系下火箭地心矢量
R_E = Rotation.L2E(display.A_L0, display.theta_T0, display.Phi_T0) * R' + display.R0_e;
R_E = R_E';

figure(3);
hold on
plot3(R_E(:,1), R_E(:,2), R_E(:,3), 'LineWidth', 3);
for i = 1:length(vec_idx)
    plot3(R_E(vec_idx(i),1), R_E(vec_idx(i),2), R_E(vec_idx(i),3), '*');
end
ellipsoid(0, 0, 0, Earth.a_e, Earth.a_e, Earth.b_e);
hold off
view(3);
axis equal
grid on
title('地心坐标系下弹道曲线');
legend('弹道曲线', '一级关机点', '二级关机点', '三级关机点');

%% 计算火箭主动段各参数数据
% 计算主动段数据长度
N_powered = size(t_powered, 1);
h_display = zeros(N_powered, 1);
v_display = zeros(N_powered, 1);
theta_v_display = zeros(N_powered, 1);
q_display = zeros(N_powered, 1);
n_display = zeros(N_powered, 1);
alpha_display = zeros(N_powered, 1);
pitch_display = zeros(N_powered, 1);
m_display = zeros(N_powered, 1);
theta_L_display = zeros(N_powered, 1);
Phi_L_display = zeros(N_powered, 1);

% 将主动段数据赋值给 display
for i = 1:size(t_powered,1)
    display = display.update(t_powered(i), X_powered(i,:));
    pitch_display(i) = rad2deg(display.pitch);
    theta_v_display(i) = rad2deg(display.theta_v);
    alpha_display(i) = rad2deg(display.alpha);
    h_display(i) = display.h * 0.001;
    v_display(i) = display.v;
    m_display(i) = display.m;
    q_display(i) = display.q * 0.001;
    theta_L_display(i) = rad2deg(display.theta_L);
    Phi_L_display(i) =  rad2deg(display.Phi_L);
    n_display(i) = display.n;
end

%% 绘制一级飞行时最大攻角、最大动压和最大法向过载
% 计算一级飞行最大攻角、最大动压和最大法向过载
[max_alpha, idx_max_alpha] = min(alpha_display(1:idx_stage1));
[max_q, idx_max_q] = max(q_display(1:idx_stage1));
[max_n, idx_max_n] = max(n_display(1:idx_stage1));

% 打印到终端
fprintf('一级飞行时最大攻角: %.2f° 在时间: %.2fs\n', max_alpha, t_powered(idx_max_alpha));
fprintf('一级飞行时最大动压: %.2fkPa 在时间: %.2fs\n', max_q, t_powered(idx_max_q));
fprintf('一级飞行时最大法向过载: %.2fg 在时间: %.2fs\n\n', max_n, t_powered(idx_max_n));

figure(4);
hold on
plot(t_powered, alpha_display, 'r');    % 绘制攻角
plot(t_powered, pitch_display, 'g');    % 绘制俯仰角
plot(t_powered, theta_v_display, 'b');  % 绘制弹道倾角
plot(t_powered(idx_max_alpha), max_alpha, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_alpha), max_alpha, sprintf('一级飞行时最大攻角:\n%.2f°', max_alpha));
hold off
xlabel('时间/s');
ylabel('角度/°');
title('攻角、俯仰角和弹道倾角随时间变化');
legend('攻角', '俯仰角', '弹道倾角');  % 添加图例
ylim([-40 90]);  % 设置y轴范围为0-90°
grid on;

%% 绘制主动段数据
figure(5);
subplot(3,3,1);
hold on
plot(t_powered, h_display);
plotShutdownPoint(t_powered, h_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('高度/km');
title('高度随时间变化');
grid on;

subplot(3,3,2);
hold on
plot(t_powered, v_display);
plotShutdownPoint(t_powered, v_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('速度/m/s');
title('速度随时间变化');
grid on;

subplot(3,3,4);
hold on
plot(t_powered, theta_L_display);
plotShutdownPoint(t_powered, theta_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理经度/°');
title('地理经度随时间变化');
grid on;

subplot(3,3,5);
hold on
plot(t_powered, Phi_L_display);
plotShutdownPoint(t_powered, Phi_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理纬度/°');
title('地理纬度随时间变化');
grid on;

subplot(3,3,7);
hold on
plot(t_powered, m_display);
plotShutdownPoint(t_powered, m_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('质量/kg');
title('质量随时间变化');
grid on;

subplot(3,3,8);
hold on
plot(t_powered, q_display);
plot(t_powered(idx_max_q), max_q, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_q), max_q, sprintf('一级飞行时最大动压:\n%.2fkPa', max_q));
plotShutdownPoint(t_powered, q_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('动压/Pa');
title('动压随时间变化');
grid on;

subplot(3,3,9);
hold on
plot(t_powered, n_display);
plot(t_powered(idx_max_n), max_n, 'o','MarkerFaceColor','k','HandleVisibility','off');
text(t_powered(idx_max_n), max_n, sprintf('一级飞行时最大法向过载:\n%.2fg', max_n));
plotShutdownPoint(t_powered, n_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('过载/g');
title('法向过载随时间变化');
grid on;

subplot(3,3,3);
hold on
plot(Inf,Inf, '*','Color','r');
plot(Inf,Inf, '*','Color','g');
plot(Inf,Inf, '*','Color','b');
legend('一级关机点', '二级关机点', '三级关机点');
hold off
axis off;

%% 计算火箭全弹道各参数数据
% 计算全弹道数据长度
N_whole = size(t_whole, 1);
h_display = zeros(N_whole, 1);
v_display = zeros(N_whole, 1);
theta_v_display = zeros(N_whole, 1);
psi_v_display = zeros(N_whole, 1);
q_display = zeros(N_whole, 1);
n_display = zeros(N_whole, 1);
alpha_display = zeros(N_whole, 1);
pitch_display = zeros(N_whole, 1);
m_display = zeros(N_whole, 1);
theta_L_display = zeros(N_whole, 1);
Phi_L_display = zeros(N_whole, 1);

% 将全弹道数据赋值给 display
for i = 1:size(t_whole,1)
    display = display.update(t_whole(i), X_whole(i,:));
    pitch_display(i) = rad2deg(display.pitch);
    theta_v_display(i) = rad2deg(display.theta_v);
    psi_v_display(i) = rad2deg(display.psi_v);
    alpha_display(i) = rad2deg(display.alpha);
    h_display(i) = display.h * 0.001;
    v_display(i) = display.v;
    m_display(i) = display.m;
    q_display(i) = display.q * 0.001;
    theta_L_display(i) = rad2deg(display.theta_L);
    Phi_L_display(i) =  rad2deg(display.Phi_L);
    n_display(i) = display.n;
end

%% 绘制全弹道数据
figure(6);
subplot(3,3,1);
hold on
plot(t_whole, h_display);
plotShutdownPoint(t_whole, h_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('高度/km');
title('高度随时间变化');
grid on;

subplot(3,3,2);
hold on
plot(t_whole, v_display);
plotShutdownPoint(t_whole, v_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('速度/m/s');
title('速度随时间变化');
grid on;

subplot(3,3,4);
hold on
plot(t_whole, theta_L_display);
plotShutdownPoint(t_whole, theta_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理经度/°');
title('地理经度随时间变化');
grid on;

subplot(3,3,5);
hold on
plot(t_whole, Phi_L_display);
plotShutdownPoint(t_whole, Phi_L_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('地理纬度/°');
title('地理纬度随时间变化');
grid on;

subplot(3,3,6);
hold on
plot(t_whole, theta_v_display);
plotShutdownPoint(t_whole, theta_v_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('弹道倾角/°');
title('弹道倾角随时间变化');
grid on;

% subplot(3,3,6);
% hold on
% plot(t_whole, psi_v_display);
% plotShutdownPoint(t_whole, psi_v_display, vec_idx);
% hold off
% xlabel('时间/s');
% ylabel('弹道偏角/°');
% title('弹道倾角随时间变化');
% grid on;

subplot(3,3,7);
hold on
plot(t_whole, m_display);
plotShutdownPoint(t_whole, m_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('质量/kg');
title('质量随时间变化');
grid on;

subplot(3,3,8);
hold on
plot(t_whole, q_display);
plotShutdownPoint(t_whole, q_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('动压/Pa');
title('动压随时间变化');
grid on;

subplot(3,3,9);
hold on
plot(t_whole, n_display);
plotShutdownPoint(t_whole, n_display, vec_idx);
hold off
xlabel('时间/s');
ylabel('过载/g');
title('法向过载随时间变化');
grid on;

subplot(3,3,3);
hold on
plot(Inf,Inf, '*','Color','r');
plot(Inf,Inf, '*','Color','g');
plot(Inf,Inf, '*','Color','b');
legend('一级关机点', '二级关机点', '三级关机点');
hold off
axis off;
end

function plotShutdownPoint(t, data, indexs)
plot(t(indexs(1)), data(indexs(1)), '*', 'Color', 'r');
plot(t(indexs(2)), data(indexs(2)), '*', 'Color', 'g');
plot(t(indexs(3)), data(indexs(3)), '*', 'Color', 'b');
end

function plotShutdownPoint3(X, indexs)
plot3(X(indexs(1),3), X(indexs(1),1), X(indexs(1),2), '*', 'Color', 'r');
plot3(X(indexs(2),3), X(indexs(2),1), X(indexs(2),2), '*', 'Color', 'g');
plot3(X(indexs(3),3), X(indexs(3),1), X(indexs(3),2), '*', 'Color', 'b');
end
