function dX = dynamic_v(t, X, rocket)
%% 状态量更新，计算各种力
rocket = rocket.update_v(t, X);

R_v = rocket.R_v();     % 气动力
g_v = rocket.g_v();     % 引力加速度
P_v = rocket.P_v();     % 推力
Fa_v = rocket.Fa_v();   % 科氏惯性力
Fe_v = rocket.Fe_v();   % 牵连惯性力

%% 运动学和动力学微分方程组
if X(1) <= 1e-3
    v = 1e-4;
else
    v = X(1);
end
if abs(X(2)-pi/2) <= 1e-3
    theta_v = pi/2 - 1e-3;
else
    theta_v = X(2);
end
psi_v = X(3);
m = X(7);

sin_theta_v = sin(theta_v);
cos_theta_v = cos(theta_v);
sin_psi_v = sin(psi_v);
cos_psi_v = cos(psi_v);

dv = (R_v(1) + P_v(1) + Fa_v(1) + Fe_v(1)) / m + g_v(1);
dtheta_v = (R_v(2) + P_v(2) + Fa_v(2) + Fe_v(2)) / (m * v) + g_v(2) / v;
if v < 1e-3
    dpsi_v = 0;
else
    dpsi_v = -(R_v(3) + P_v(3) + Fa_v(3) + Fe_v(3)) / (m * v * cos_theta_v) - g_v(3) / (v * cos_theta_v);
end
dx = v * cos_theta_v * cos_psi_v;
dz = -v * cos_theta_v * sin_psi_v;
dy = v * sin_theta_v;
if 0 <= t && t < rocket.t_stage(1)
    dm = -rocket.dm(1);
elseif t < rocket.t_stage(1) + rocket.t_stage(2)
    dm = -rocket.dm(2);
elseif t < rocket.t_stage(1) + rocket.t_stage(2) + rocket.t_stage(3)
    dm = -rocket.dm(3);
else
    dm = 0;
end

dX = [dv;dtheta_v;dpsi_v;dx;dy;dz;dm];
end
