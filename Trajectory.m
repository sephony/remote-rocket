classdef Trajectory
    properties
        rocket;             % 火箭对象
        
        X_powered;           % 主动段数据
        t_powered;           % 主动段时间
        X_whole;            % 全弹道数据
        t_whole;            % 全弹道时间
        
        % 微分方程参数设置
        step;               % 外部循环步长
        N;                  % 预分配内存
        X_count;            % 状态量矩阵
        t_count;            % 时间矩阵
        index;              % 索引变量
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
        %% 主动段弹道计算
        function obj = calc_powered(obj)
            % 主动段弹道计算
            for t = 0: obj.step: (obj.rocket.t_stage(2) + obj.rocket.t_stage(1) + obj.rocket.t_stage(3) - obj.step)
                [t_t, X_t] = ode45(@dynamic, [t; t+obj.step], obj.rocket.X, [], obj.rocket);
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
        function obj = calc_passive(obj)
            % 被动段弹道计算
            for t = (obj.rocket.t_stage(1) + obj.rocket.t_stage(2) + obj.rocket.t_stage(3)): obj.step: 100000
                [t_t, X_t] = ode45(@dynamic, [t, t + obj.step], obj.rocket.X, [], obj.rocket);
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
        % %% 计算火箭状态
        % function obj = calc_state(obj)
        %     % 计算火箭状态
        %     obj.rocket = obj.rocket.calc_state();
        % end
        % %% 计算火箭轨迹
        % function obj = calc_trajectory(obj)
        %     % 计算火箭轨迹
        %     obj.rocket = obj.rocket.calc_trajectory();
        % end
    end
end
