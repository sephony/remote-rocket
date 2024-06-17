classdef Trajectory
    properties
        rocket;             % 火箭对象
        
        X_powered;          % 主动段数据
        t_powered;          % 主动段时间
        X_whole;            % 全弹道数据
        t_whole;            % 全弹道时间
        
        % 微分方程参数设置
        step;               % 外部循环步长
        N;                  % 预分配内存
        X_count;            % 状态量矩阵
        t_count;            % 时间矩阵
        index;              % 索引变量
        method;             % 动力学微分方程方法
        odefun;             % 微分方程函数
    end
    methods
        %% 构造函数
        function obj = Trajectory(rocket)
            obj.rocket = rocket;
            obj.step = 1;
            obj.N = floor(500000 / obj.step);
            obj.X_count = zeros(obj.N, 7);
            obj.t_count = zeros(obj.N, 1);
            obj.index = 1;
        end
        %% 选择动力学微分方程方法
        function obj = choose_method(obj, dynamic_type)
            obj.method = dynamic_type;
            if dynamic_type == "launch"
                obj.odefun = @(t, X, rocket) obj.dynamic_L(t, X, rocket);
            elseif dynamic_type == "velocity"
                obj.odefun = @(t, X, rocket) obj.dynamic_V(t, X, rocket);
            else
                error('请选择正确的动力学微分方程方法！(launch or velocity)');
            end
        end
        %% 动力学微分方程
        % 发射系下动力学微分方程
        function dX = dynamic_L(~, t, X, rocket)
            %%状态量更新，计算各种力
            rocket = rocket.update(t, X);
            
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
        function dX = dynamic_V(~, t, X, rocket)
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
        %% 主动段弹道计算
        function obj = calc_powered(obj, dynamic_type)
            obj = obj.choose_method(dynamic_type);
            % 主动段弹道计算
            for t = 0: obj.step: (obj.rocket.t_stage(2) + obj.rocket.t_stage(1) + obj.rocket.t_stage(3) - obj.step)
                [t_t, X_t] = ode45(@obj.odefun, [t; t+obj.step], obj.rocket.X, [], obj.rocket);
                num_rows = size(X_t, 1);% 计算 X_t 的行数
                % 将 X_t 的数据插入到 X_count 中
                obj.X_count(obj.index:(obj.index+num_rows-1), :) = X_t;
                % 将 t_t 的数据插入到 t_count 中
                obj.t_count(obj.index:(obj.index+num_rows-1)) = t_t;
                % 更新索引变量
                obj.index = obj.index + num_rows;
                
                obj.rocket = obj.rocket.update(t+1, obj.X_count(obj.index-1, :));
            end
            obj.X_powered = obj.X_count(1:obj.index-1, :);
            obj.t_powered = obj.t_count(1:obj.index-1);
        end
        %% 被动段弹道计算
        function obj = calc_passive(obj, dynamic_type)
            obj = obj.choose_method(dynamic_type);
            % 被动段弹道计算
            for t = (obj.rocket.t_stage(1) + obj.rocket.t_stage(2) + obj.rocket.t_stage(3)): obj.step: 100000
                [t_t, X_t] = ode45(@obj.odefun, [t, t + obj.step], obj.rocket.X, [], obj.rocket);
                num_rows = size(X_t, 1);
                obj.X_count(obj.index:(obj.index+num_rows-1), :) = X_t;
                obj.t_count(obj.index:(obj.index+num_rows-1)) = t_t;
                obj.index = obj.index + num_rows;
                obj.rocket = obj.rocket.update(t+1, obj.X_count(obj.index-1, :));
                if (obj.rocket.h > 0) && ((obj.rocket.h / obj.rocket.v) < (2 * obj.step))
                    fprintf('导弹打击时间为：%.2fs （相对误差 < %.2fs)\n',t, obj.step);
                    fprintf('导弹打击点经度：%.2f°, 纬度：%.2f°\n\n', rad2deg(obj.rocket.theta_L), rad2deg(obj.rocket.Phi_L));
                    break;
                end
            end
            % 截取全弹道数据
            obj.X_whole = obj.X_count(1:obj.index-1, :);
            obj.t_whole = obj.t_count(1:obj.index-1);
        end
    end
end
