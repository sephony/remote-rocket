clc; clear; close all;
tic;
%% 火箭发射点参数
A_L0 = deg2rad(-20);      % 发射点地理方位角
theta_L0 = deg2rad(60);   %（东经为正，西经为负）
Phi_L0 = deg2rad(-30);    %（北纬为正，南纬为负）

pitch_data = load('data/FiC.txt');   % 读取俯仰角飞行程序数据
% 创建火箭对象
rocket = Rocket(A_L0, theta_L0, Phi_L0, pitch_data);

%% 弹道计算
rocket = rocket.solve();

%% 数据可视化
tStart_visualize = tic;
rocket = rocket.plot();

toc;
