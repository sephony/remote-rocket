function dX = dynamic(t, X, rocket)
%% 状态量更新，计算各种力
rocket = rocket.update(t, X);

g_L = rocket.g_L();     % 发射坐标系下的引力加速度
R_L = rocket.R_L();     % 发射系下气动力
P_L = rocket.P_L();     % 推力
Fa_L = rocket.Fa_L();   % 科氏惯性力
Fe_L = rocket.Fe_L();   % 牵连惯性力

%% 运动学和动力学微分方程组
v_x = X(4);
v_y = X(5);
v_z = X(6);
m = X(7);

dx = v_x;
dy = v_y;
dz = v_z;
dv_x = g_L(1) + (R_L(1) + P_L(1) + Fa_L(1) + Fe_L(1)) / m;
dv_y = g_L(2) + (R_L(2) + P_L(2) + Fa_L(2) + Fe_L(2)) / m;
dv_z = g_L(3) + (R_L(3) + P_L(3) + Fa_L(3) + Fe_L(3)) / m;
if 0 <= t && t < rocket.t_stage(1)
    dm = -rocket.dm(1);
elseif t < rocket.t_stage(1) + rocket.t_stage(2)
    dm = -rocket.dm(2);
elseif t < rocket.t_stage(1) + rocket.t_stage(2) + rocket.t_stage(3)
    dm = -rocket.dm(3);
else
    dm = 0;
end

dX = [dx;dy;dz;dv_x;dv_y;dv_z;dm];
