% File: main.m
% Author: 乔栋
% Date: 2024-06-24
% Subject: 火箭弹道仿真主程序
% Description:  本程序为火箭弹道仿真脚本，包括火箭发射点参数设置、火箭对象创建
%               、火箭弹道计算、数据可视化等功能。

clc; clear; close all;
tic;
%% 火箭发射点参数
A_L0 = -20;      % 发射点地理方位角
theta_L0 = 60;   %（东经为正，西经为负）
Phi_L0 = -30;    %（北纬为正，南纬为负）

pitch_data_path = 'data/FiC.txt';
% 创建火箭对象
rocket = Rocket(A_L0, theta_L0, Phi_L0, pitch_data_path);
% 设置火箭主动段的动力学模型
% rocket = rocket.set_powered_method("launch");
rocket = rocket.set_powered_method("velocity");
%% 弹道计算
rocket = rocket.solve();

%% 数据可视化
rocket = rocket.plot();

toc;
