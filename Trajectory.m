classdef Trajectory
    properties
        rocket_temp;        % 火箭对象
        
        X_powered;          % 主动段数据
        t_powered;          % 主动段时间
        X_passive;          % 被动段数据
        t_passive;          % 被动段时间
        X_whole;            % 全弹道数据（发射系）
        t_whole;            % 全弹道时间（发射系）
        % 微分方程参数设置
        step;               % 外部循环步长
        N;                  % 预分配内存
        X_count;            % 状态量矩阵
        t_count;            % 时间矩阵
        method;             % 动力学微分方程方法
        odefun;             % 微分方程函数
    end
    methods
        %% 构造函数
        function obj = Trajectory(rocket)
            obj.rocket_temp = rocket;
            obj.step = 1;
            obj.N = floor(500000 / obj.step);
            obj.X_count = zeros(obj.N, 7);
            obj.t_count = zeros(obj.N, 1);
        end
        %% 选择动力学微分方程方法
        function obj = choose_method(obj, dynamic_type)
            obj.method = dynamic_type;
            if dynamic_type == "launch"
                obj.odefun = @(t, X, rocket) obj.dynamic_L(t, X, rocket);
                obj.rocket_temp = obj.rocket_temp.choose_method(dynamic_type);
            elseif dynamic_type == "velocity"
                obj.odefun = @(t, X, rocket) obj.dynamic_v(t, X, rocket);
                obj.rocket_temp = obj.rocket_temp.choose_method(dynamic_type);
            else
                error('请选择正确的动力学微分方程方法！(launch or velocity)');
            end
        end
        %% 动力学微分方程
        % 发射系下动力学微分方程
        function dX = dynamic_L(~, t, X, rocket)
            %%状态量更新，计算各种力
            rocket = rocket.update_L(t, X);
            
            g_L = rocket.g_L();     % 发射坐标系下的引力加速度
            R_L = rocket.R_L();     % 发射系下气动力
            P_L = rocket.P_L();     % 推力
            Fa_L = rocket.Fa_L();   % 科氏惯性力
            Fe_L = rocket.Fe_L();   % 牵连惯性力
            
            % 运动学和动力学微分方程组
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
        end
        % 速度系下动力学微分方程
        function dX = dynamic_v(~, t, X, rocket)
            rocket = rocket.update_v(t, X);
            
            R = rocket.R_v();       % 速度系下气动力
            g_L = rocket.g_L();     % 发射坐标系下的引力加速度
            P_L = rocket.P_L();     % 推力
            Fa_L = rocket.Fa_L();   % 科氏惯性力
            Fe_L = rocket.Fe_L();   % 牵连惯性力
            
            g_v = Rotation.L2V(rocket.sigma, rocket.psi_v, rocket.theta_v) * g_L;
            P_v = Rotation.L2V(rocket.sigma, rocket.psi_v, rocket.theta_v) * P_L;
            Fa_v = Rotation.L2V(rocket.sigma, rocket.psi_v, rocket.theta_v) * Fa_L;
            Fe_v = Rotation.L2V(rocket.sigma, rocket.psi_v, rocket.theta_v) * Fe_L;
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
            
            dv = (R(1) + P_v(1) + Fa_v(1) + Fe_v(1)) / m + g_v(1);
            dtheta_v = (R(2) + P_v(2) + Fa_v(2) + Fe_v(2)) / (m * v) + g_v(2) / v;
            if v < 1e-3
                dpsi_v = 0;
            else
                dpsi_v = -(R(3) + P_v(3) + Fa_v(3) + Fe_v(3)) / (m * v * cos_theta_v) - g_v(3) / (v * cos_theta_v);
            end
            dx = v * cos_theta_v * cos_psi_v;
            dy = v * sin_theta_v;
            dz = -v * cos_theta_v * sin_psi_v;
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
        %% 主动段弹道计算
        function obj = calc_powered(obj, dynamic_type)
            obj = obj.choose_method(dynamic_type);
            rocket = obj.rocket_temp;
            index = 1;
            % 主动段弹道计算
            for t = 0: obj.step: (rocket.t_stage(2) + rocket.t_stage(1) + rocket.t_stage(3) - obj.step)
                [t_t, X_t] = ode45(@obj.odefun, [t; t+obj.step], rocket.X, [], rocket);
                num_rows = size(X_t, 1);% 计算 X_t 的行数
                % 将 X_t 的数据插入到 X_count 中
                obj.X_count(index:(index+num_rows-1), :) = X_t;
                % 将 t_t 的数据插入到 t_count 中
                obj.t_count(index:(index+num_rows-1)) = t_t;
                % 更新索引变量
                index = index + num_rows;
                
                rocket = rocket.update(t+1, obj.X_count(index-1, :));
            end
            obj.rocket_temp = rocket;
            % 截取主动段数据
            obj.X_powered = obj.X_count(1:index-1, :);
            obj.t_powered = obj.t_count(1:index-1);
        end
        %% 被动段弹道计算
        function obj = calc_passive(obj, dynamic_type)
            obj = obj.choose_method(dynamic_type);
            rocket = obj.rocket_temp;
            index = 1;
            % 如果主动段采用速度系，则将主动段的数据转换为发射系下的数据
            if rocket.powered_method == "velocity"
                rocket.X = [rocket.R_launch; rocket.V_launch; rocket.m];
            end
            % 被动段弹道计算
            for t = (rocket.t_stage(1) + rocket.t_stage(2) + rocket.t_stage(3)): obj.step: 100000
                [t_t, X_t] = ode45(@obj.odefun, [t, t + obj.step], rocket.X, [], rocket);
                num_rows = size(X_t, 1);
                obj.X_count(index:(index+num_rows-1), :) = X_t;
                obj.t_count(index:(index+num_rows-1)) = t_t;
                index = index + num_rows;
                rocket = rocket.update(t+1, obj.X_count(index-1, :));
                if (rocket.h > 0) && ((rocket.h / rocket.v) < (2 * obj.step))
                    fprintf('导弹打击时间为：%.2fs （相对误差 < %.2fs)\n',t, obj.step);
                    fprintf('导弹打击点经度：%.2f°, 纬度：%.2f°\n\n', rad2deg(rocket.theta_L), rad2deg(rocket.Phi_L));
                    break;
                end
            end
            obj.rocket_temp = rocket;
            % 截取全弹道数据
            obj.X_passive = obj.X_count(1:index-1, :);
            obj.t_passive = obj.t_count(1:index-1);
        end
        %% 全弹道数据合并
        function obj = merge_data(obj)
            % 转换主动段数据
            if(obj.rocket_temp.powered_method == "velocity")
                obj.X_powered = obj.data_v2L(obj.X_powered, obj.t_powered);
            end
            obj.X_whole = [obj.X_powered; obj.X_passive];
            obj.t_whole = [obj.t_powered; obj.t_passive];
        end
        
        % 数据转换
        function X_L = data_v2L(obj, X, t)
            % 计算主动段数据长度
            n = size(t, 1);
            X_L = zeros(n, 7);
            rocket = obj.rocket_temp;
            for i = 1:size(t,1)
                rocket = rocket.update_v(t(i), X(i,:));
                X_L(i,:) = [rocket.R_launch', rocket.V_launch', rocket.m];
            end
        end
    end
end
