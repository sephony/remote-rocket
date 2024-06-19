function dX = dynamic_v(t, X, rocket)
%% 状态量更新，计算各种力
rocket = rocket.update(t, X);

R = rocket.R_v();     % 发射系下气动力
% D = R(1);
%% 运动学和动力学微分方程组
v = X(1);
theta_v = X(2);
psi_v = X(3);
m = X(7);

sin_theta_v = sin(theta_v);
cos_theta_v = cos(theta_v) + 0.1;
sin_psi_v = sin(psi_v);
cos_psi_v = cos(psi_v);

dv = R(1)/m - Earth.g_0 * sin_theta_v;
dtheta_v = - R(2) / (m * v) - Earth.g_0 * cos_theta_v / v;
dpsi_v = - R(3) / (m * v * cos_theta_v);
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
