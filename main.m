clc; clear; close all;
tic;
%% 火箭发射点参数
A_L0 = deg2rad(-20);      % 发射点地理方位角
theta_L0 = deg2rad(60);   %（东经为正，西经为负）
Phi_L0 = deg2rad(-30);    %（北纬为正，南纬为负）

pitch_data = load('data/FiC.txt');   % 读取俯仰角飞行程序数据
% 创建火箭对象
rocket = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);
% 设置火箭主动段的动力学模型
% rocket = rocket.set_powered_method("launch");
rocket = rocket.set_powered_method("velocity");
%% 弹道计算
fprintf('正在解算弹道...\n');
tStart_solve = tic;
rocket = rocket.solve();
tEnd_solve = toc(tStart_solve);
fprintf('弹道解算的时间是 %.2f 秒\n\n', tEnd_solve);

%% 数据可视化
fprintf('正在可视化火箭参数...\n');
tStart_visualize = tic;
rocket = rocket.plot();
tEnd_visualize = toc(tStart_visualize);
fprintf('数据可视化用的时间是 %.2f 秒\n\n', tEnd_visualize);

toc;
