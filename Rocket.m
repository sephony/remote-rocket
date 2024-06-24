classdef Rocket
    properties(Constant)
        %% 火箭参数
        m_stage = [18800 5400 1300];        % 各级火箭质量
        d_stage = [2.0 2.0 1.0];            % 各级火箭直径
        P_stage = [485000 120000 35000];    % 各级火箭推力
        Isp_stage = [300 300 300];          % 各级火箭比冲
        t_stage = [72 71 64];               % 各级工作时间
        
    end
    properties
        %% 火箭状态
        t;                  % 火箭飞行时间
        X;                  % 火箭状态量
        R_launch;           % 火箭位置矢量（发射坐标系下）
        V_launch;           % 火箭速度矢量（发射坐标系下）
        m;                  % 火箭质量
        dm;                 % 各级发动机秒耗量
        
        pitch;              % 火箭俯仰角
        yaw;                % 火箭偏航角
        roll;               % 火箭滚转角
        theta_v;            % 火箭弹道倾角
        psi_v;              % 火箭弹道偏角
        sigma;              % 火箭倾侧角
        alpha;              % 火箭攻角
        beta;               % 火箭侧滑角
        
        Rc_e;               % 火箭地心矢径(地心坐标系下)
        Rc_L;               % 火箭地心矢径(发射坐标系下)
        r;                  % 火箭地心距离
        r_Ue;               % 火箭星下点地心距
        h;                  % 火箭海拔高度
        v;                  % 火箭速度大小
        
        theta_L;            % 火箭地理经度
        Phi_L;              % 火箭地理纬度
        theta_T;            % 火箭天文经度
        Phi_T;              % 火箭天文纬度
        
        q;                  % 动压
        n;                  % 过载
    end
    properties
        %% 发射点参数
        A_L0;               % 发射点地理方位角
        theta_L0;           % 发射点地理经度
        Phi_L0;             % 发射点地理纬度
        theta_T0;           % 发射点天文经度
        Phi_T0;             % 发射点天文纬度
        
        r0;                 % 发射点地心距离
        R0_e;               % 发射点地心矢径(地心坐标系下)
        R0_L;               % 发射点地心矢径（发射坐标系下）
        
        path;               % 程序俯仰角数据路径
        data;               % 程序俯仰角数据
        
        trajectory;         % 弹道对象
        plotter;            % 可视化对象
        powered_method;     % 主动段动力学微分方程方法
        update;             % 更新状态函数
    end
    
    
    methods
        %% 构造函数
        function obj = Rocket(A_L0, theta_L0, Phi_L0, pitch_data_path, varargin)
            % 发射点参数,将发射点的地理经纬度转换为天文经纬度
            obj.A_L0 = deg2rad(A_L0);
            obj.theta_L0 = deg2rad(theta_L0);
            obj.Phi_L0 = deg2rad(Phi_L0);
            obj.theta_T0 = Earth.theta_L2T(obj.theta_L0);
            obj.Phi_T0 = Earth.Phi_L2T(obj.Phi_L0);
            obj.path = pitch_data_path;
            % 发射点地心距离
            obj.r0 = Earth.a_e * (1 - Earth.e_E)/sqrt((sind(Phi_L0))^2 + (1-Earth.e_E)^2 * (cosd(Phi_L0))^2);
            obj.R0_e = obj.r0 * [cosd(Phi_L0) * cosd(theta_L0);cosd(Phi_L0) * sind(theta_L0);sind(Phi_L0)];
            obj.R0_L = Rotation.L2E(obj.A_L0, obj.theta_L0, obj.Phi_L0)' * obj.R0_e;
            % 计算火箭各级发动机秒耗量
            obj.dm = [Rocket.P_stage(1) / (Earth.g_0 * Rocket.Isp_stage(1)), Rocket.P_stage(2) / (Earth.g_0 * Rocket.Isp_stage(2)), Rocket.P_stage(3) / (Earth.g_0 * Rocket.Isp_stage(3))];
            % 火箭姿态初始化
            obj.yaw = 0;
            obj.roll = 0;
            obj.sigma = 0;
            obj.beta = 0;
            % 读取俯仰角飞行程序数据
            obj.data = load(obj.path);
            absolutePath = fullfile(pwd, obj.path);
            % 创建一个输入解析器
            p = inputParser;
            % 定义期望的参数
            addOptional(p, 'print_flag', true);
            addOptional(p, 'powered_method', 'launch');
            % 解析输入参数
            parse(p, varargin{:});
            % 提取参数值
            print_flag = p.Results.print_flag;
            obj.powered_method = p.Results.powered_method;
            if print_flag == true
                fprintf('-------------------------------------------------------------\n');
                fprintf('                 ** 火箭发射点及其他参数 **\n');
                fprintf('-------------------------------------------------------------\n');
                fprintf('\t发射点地理方位角\t\t\t %.2f°\n', A_L0);
                fprintf('\t发射点地理经度\t\t\t\t %.2f°\n', theta_L0);
                fprintf('\t发射点地理纬度\t\t\t\t %.2f°\n', Phi_L0);
                fprintf('\t发射点天文经度\t\t\t\t %.2f°\n', rad2deg(obj.theta_T0));
                fprintf('\t发射点天文纬度\t\t\t\t %.2f°\n', rad2deg(obj.Phi_T0));
                fprintf('\t发射点地心距离\t\t\t\t %.2f km\n', obj.r0 / 1e3);
                fprintf('\t发射点地心矢径\t\t\t\t [%.2f, %.2f, %.2f] km\n', obj.R0_e / 1e3);
                fprintf('-------------------------------------------------------------\n');
                fprintf('\t各级发动机秒耗量\t\t\t [%.2f, %.2f, %.2f] kg/s\n', obj.dm);
                fprintf('\t俯仰角数据路径\t\t\t\t %s\n', absolutePath);
            end
            obj = obj.set_powered_method(obj.powered_method, 'print_flag', false);
        end
        
        % 设置火箭主动段的动力学模型并更新状态
        function obj = set_powered_method(obj, powered_method, varargin)
            obj.powered_method = powered_method;
            X0_L = [0; 0; 0; 0; 0; 0; Rocket.m_stage(1)];
            X0_v = [0; pi/2; 0; 0; 0; 0; Rocket.m_stage(1)];
            if powered_method == "launch"
                obj.X = X0_L;
                obj.update = @(t, X) obj.update_L(t, X);
            elseif powered_method == "velocity"
                obj.X = X0_v;
                obj.update = @(t, X) obj.update_v(t, X);
            else
                obj.powered_method = "error";
                error('火箭动力学微分方程方法错误, 请选择正确的动力学微分方程方法！(launch or velocity)');
            end
            obj = obj.update(0, obj.X);% 更新状态
            % 创建一个输入解析器
            p = inputParser;
            % 定义期望的参数
            addOptional(p, 'print_flag', true);
            % 解析输入参数
            parse(p, varargin{:});
            % 提取参数值
            print_flag = p.Results.print_flag;
            if print_flag == true
                fprintf('\t主动段动力学微分方程方法\t\t %s\n', obj.powered_method);
                fprintf('-------------------------------------------------------------\n');
            end
        end
        
        function obj = choose_method(obj, dynamic_type)
            if dynamic_type == "launch"
                obj.update = @(t, X) obj.update_L(t, X);
            elseif dynamic_type == "velocity"
                obj.update = @(t, X) obj.update_v(t, X);
            else
                error('火箭动力学微分方程方法错误, 请选择正确的动力学微分方程方法！(launch or velocity)');
            end
        end
        
        function obj = solve(obj)
            fprintf('* 正在解算弹道...\n');
            tStart_solve = tic;
            obj.trajectory = Trajectory(obj);   % 创建弹道对象
            tStart_powered = tic;
            obj.trajectory = obj.trajectory.calc_powered(obj.powered_method);
            tEnd_powered = toc(tStart_powered);
            fprintf('  主动段弹道解算的时间是 %.2f 秒\n', tEnd_powered);
            tStart_passive = tic;
            obj.trajectory = obj.trajectory.calc_passive("launch");
            tEnd_passive = toc(tStart_passive);
            fprintf('  被动段弹道解算的时间是 %.2f 秒\n', tEnd_passive);
            obj.trajectory = obj.trajectory.merge_data();
            tEnd_solve = toc(tStart_solve);
            fprintf('  弹道解算的时间是 %.2f 秒\n\n', tEnd_solve);
            
            t_end = obj.trajectory.t_whole(end);
            obj = obj.choose_method("launch");
            obj = obj.update(t_end, obj.trajectory.X_whole(end, :)');
            fprintf('  导弹打击时间为：%.2fs （相对误差 < %.2fs)\n',t_end, obj.trajectory.step);
            fprintf('  导弹打击点经度：%.4f°, 纬度：%.4f°\n\n', rad2deg(obj.theta_L), rad2deg(obj.Phi_L));
        end
        
        function obj = plot(obj)
            fprintf('* 正在可视化火箭参数...\n');
            tStart_visualize = tic;
            obj.plotter = Plotter(obj);
            obj.plotter = obj.plotter.plot_trajectoryCurve();
            obj.plotter = obj.plotter.plot_poweredData();
            obj.plotter = obj.plotter.plot_wholeData();
            tEnd_visualize = toc(tStart_visualize);
            fprintf('  数据可视化用的时间是 %.2f 秒\n\n', tEnd_visualize);
        end
        
        %% 更新状态(根据变化后的X更新状态量)
        function obj = update_L(obj, t, X)
            obj.t = t;
            obj.X = X(:); % 将 X 转换为列向量
            obj.R_launch = obj.X(1:3);
            obj.V_launch = obj.X(4:6);
            % 如果到达级间分离时间，更新火箭质量
            if(obj.t == Rocket.t_stage(1))
                obj.X(7) = Rocket.m_stage(2);
            elseif obj.t == Rocket.t_stage(1) + Rocket.t_stage(2)
                obj.X(7) = Rocket.m_stage(3);
            end
            obj.m = obj.X(7);
            
            obj.Rc_e = obj.get_Rc_e();
            obj.Rc_L = obj.get_Rc_L();
            obj.r = norm(obj.Rc_e);
            obj.v = norm(obj.V_launch);
            % 计算弹道倾角和弹道偏角
            if obj.V_launch(2) >= 0             %上升段
                if  obj.V_launch(1) < 1e-4    %火箭起飞判定
                    obj.theta_v = pi/2;
                    obj.psi_v = 0;
                else
                    obj.theta_v = atan(obj.V_launch(2) / obj.V_launch(1));
                    obj.psi_v = atan(-obj.V_launch(3) / (cos(obj.theta_v) * obj.V_launch(1) + sin(obj.theta_v) * (obj.V_launch(2) + 1e-8)));
                end
            else
                if abs(obj.V_launch(1)) < 1e-4%防止除0，平滑过渡-90°
                    obj.theta_v = -pi/2;
                    obj.psi_v = 0;
                else
                    obj.theta_v = atan(obj.V_launch(2) / obj.V_launch(1));
                    if obj.V_launch(1) < 0      %实际弹道倾角在-90°到-180°之间
                        obj.theta_v = obj.theta_v - pi;
                    end
                    obj.psi_v = atan(-obj.V_launch(3) / (cos(obj.theta_v) * obj.V_launch(1) + sin(obj.theta_v) * (obj.V_launch(2) + 1e-8)));
                end
            end
            obj.pitch = Rocket.interpolation(t, obj.data);
            obj.alpha = obj.pitch - obj.theta_v;
            obj.beta = obj.yaw - obj.psi_v;
            
            obj.theta_L = atan2(obj.Rc_e(2), obj.Rc_e(1));
            obj.Phi_L = asin(obj.Rc_e(3) / obj.r);
            obj.theta_T = Earth.theta_L2T(obj.theta_L);
            obj.Phi_T = Earth.Phi_L2T(obj.Phi_L);
            
            obj.r_Ue = Earth.a_e * (1 - Earth.e_E)/sqrt((sin(obj.Phi_T))^2 + (1-Earth.e_E)^2 * (cos(obj.Phi_T))^2);
            obj.h = obj.r - obj.r_Ue;
            
            obj.q = obj.get_q();
            obj.n = obj.get_n();
            obj.update = @(t, X) obj.update_L(t, X);
        end
        
        function obj = update_v(obj, t, X)
            obj.t = t;
            obj.X = X(:); % 将 X 转换为列向量
            obj.v = obj.X(1);
            obj.theta_v = obj.X(2);
            obj.psi_v = obj.X(3);
            obj.R_launch = obj.X(4:6);
            obj.V_launch = [obj.v * cos(obj.theta_v) * cos(obj.psi_v); obj.v * sin(obj.theta_v); -obj.v * cos(obj.theta_v) * sin(obj.psi_v)];
            % 如果到达级间分离时间，更新火箭质量
            if(obj.t == Rocket.t_stage(1))
                obj.X(7) = Rocket.m_stage(2);
            elseif obj.t == Rocket.t_stage(1) + Rocket.t_stage(2)
                obj.X(7) = Rocket.m_stage(3);
            end
            obj.m = obj.X(7);
            
            obj.Rc_e = obj.get_Rc_e();
            obj.Rc_L = obj.get_Rc_L();
            obj.r = norm(obj.Rc_e);
            
            obj.pitch = Rocket.interpolation(t, obj.data);
            obj.alpha = obj.pitch - obj.theta_v;
            obj.beta = obj.yaw - obj.psi_v;
            
            obj.theta_L = atan2(obj.Rc_e(2), obj.Rc_e(1));
            obj.Phi_L = asin(obj.Rc_e(3) / obj.r);
            obj.theta_T = Earth.theta_L2T(obj.theta_L);
            obj.Phi_T = Earth.Phi_L2T(obj.Phi_L);
            
            obj.r_Ue = Earth.a_e * (1 - Earth.e_E)/sqrt((sin(obj.Phi_T))^2 + (1-Earth.e_E)^2 * (cos(obj.Phi_T))^2);
            obj.h = obj.r - obj.r_Ue;
            
            obj.q = obj.get_q();
            obj.n = obj.get_n();
            obj.update = @(t, X) obj.update_v(t, X);
        end
        
        %% 力计算
        % 火箭受到的引力加速度（北天东地坐标系下）
        function g_N = g_N(obj)
            g_r = -Earth.mu / obj.r^2 * (1 + 1.5 * Earth.J2 * (Earth.a_e / obj.r)^2 * (1 - 5 * sin(obj.Phi_T)^2));
            g_omega = -3 * Earth.mu / obj.r^2 * Earth.J2 * (Earth.a_e / obj.r)^2 * sin(obj.Phi_T);
            g_N = [g_r; g_omega];
        end
        
        % 火箭受到的引力加速度（发射坐标系下）
        function g_L = g_L(obj)
            g_N = obj.g_N();
            g_L = g_N(1) / obj.r * obj.Rc_L + g_N(2) * Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0)' * [0; 0; 1];
        end
        
        % 火箭受到的引力加速度（速度坐标系下）
        function g_v = g_v(obj)
            g_v = Rotation.L2V(obj.sigma, obj.psi_v, obj.theta_v) * obj.g_L();
        end
        
        % 火箭受到的气动力（速度坐标系下）
        function R_v = R_v(obj)
            % 火箭直径
            if 0 <= obj.t && obj.t < Rocket.t_stage(1)
                d = Rocket.d_stage(1);
            elseif obj.t < Rocket.t_stage(1) + Rocket.t_stage(2)
                d = Rocket.d_stage(2);
            elseif Rocket.t_stage(1) + Rocket.t_stage(2) <= obj.t
                d = Rocket.d_stage(3);
            end
            S = pi * (d / 2) ^ 2;     % 特征面积
            
            % 计算气动参数
            if(0 <= obj.h && obj.h <= 40000)
                C_x = 0.09;
                C_y = 0.13;
            elseif(40000 < obj.h && obj.h <= 60000)
                C_x = 0.04;
                C_y = 0.08;
            else
                C_x = 0;
                C_y = 0;
            end
            C_z = 0;
            
            D = C_x * obj.q * S;
            L = C_y * obj.q * S;
            Z = C_z * obj.q * S;
            
            R_v = [-D; L; Z];
        end
        
        % 火箭受到的气动力（发射坐标系下）
        function R_L = R_L(obj)
            % 气动力在速度坐标系下的分量
            R = obj.R_v();
            % 气动力在发射坐标系下的分量
            R_L = Rotation.L2V(obj.sigma, obj.psi_v, obj.theta_v)' * R;
        end
        
        % 火箭受到的气动力（体坐标系下）
        function R_B = R_B(obj)
            R_B = Rotation.L2B(obj.pitch, obj.yaw, obj.roll) * obj.R_L();
        end
        
        % 火箭受到的推力（体坐标系下）
        function P_B = P_B(obj)
            if 0 <= obj.t && obj.t < Rocket.t_stage(1)
                P_B = [Rocket.P_stage(1); 0; 0];
            elseif obj.t < Rocket.t_stage(1) + Rocket.t_stage(2)
                P_B = [Rocket.P_stage(2); 0; 0];
            elseif obj.t < Rocket.t_stage(1) + Rocket.t_stage(2) + Rocket.t_stage(3)
                P_B = [Rocket.P_stage(3); 0; 0];
            else
                P_B = [0; 0; 0];    %被动段无推力
            end
        end
        
        % 火箭受到的推力（发射坐标系下）
        function P_L = P_L(obj)
            P_L = Rotation.L2B(obj.pitch, obj.yaw, obj.roll)' * obj.P_B();
        end
        
        % 火箭受到的推力（速度坐标系下）
        function P_v = P_v(obj)
            P_v = Rotation.V2B(obj.alpha, obj.beta)' * obj.P_B();
            % P_v = Rotation.L2V(obj.sigma, obj.psi_v, obj.theta_v) * obj.P_L();
        end
        
        % 火箭受到的科氏惯性力（发射坐标系下）
        function Fa_L = Fa_L(obj)
            % 地球自转角速度矢量（发射坐标系下）
            omega_L = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0)' * [0; 0; Earth.omega];
            Fa_L = -2 * obj.X(7) * cross(omega_L, obj.V_launch);
        end
        
        % 火箭受到的科氏惯性力（速度坐标系下）
        function Fa_v = Fa_v(obj)
            Fa_v = Rotation.L2V(obj.sigma, obj.psi_v, obj.theta_v) * obj.Fa_L();
        end
        
        % 火箭受到的科氏惯性力（体坐标系下）
        function Fa_B = Fa_B(obj)
            Fa_B = Rotation.L2B(obj.pitch, obj.yaw, obj.roll) * obj.Fa_L();
        end
        
        % 火箭受到的牵连惯性力（发射坐标系下）
        function Fe_L = Fe_L(obj)
            % 地球自转角速度矢量（发射坐标系下）
            omega_L = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0)' * [0; 0; Earth.omega];
            Fe_L = -obj.X(7) * cross(omega_L, cross(omega_L, obj.V_launch));
        end
        
        % 火箭受到的牵连惯性力（速度坐标系下）
        function Fe_v = Fe_v(obj)
            Fe_v = Rotation.L2V(obj.sigma, obj.psi_v, obj.theta_v) * obj.Fe_L();
        end
        
        % 火箭受到的牵连惯性力（体坐标系下）
        function Fe_B = Fe_B(obj)
            Fe_B = Rotation.L2B(obj.pitch, obj.yaw, obj.roll) * obj.Fe_L();
        end
        
        function init_para = get_init_para(obj)
            % 初始化参数作为结构体返回
            init_para.A_L0 = rad2deg(obj.A_L0);
            init_para.theta_L0 = rad2deg(obj.theta_L0);
            init_para.Phi_L0 = rad2deg(obj.Phi_L0);
            init_para.pitch_data_path = obj.path;
        end
    end
    
    
    methods(Access = private)
        % 火箭地心矢径（地心坐标系下）
        function Rc_e = get_Rc_e(obj)
            Rc_e = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0) * obj.R_launch + obj.R0_e;
        end
        
        % 火箭地心矢径(发射坐标系下)
        function Rc_L = get_Rc_L(obj)
            Rc_L = obj.R_launch + obj.R0_L;
        end
        
        % 计算动压
        function q = get_q(obj)
            rho = 1.225 * exp(-1.406e-4 * obj.h);   % 大气密度
            q = 0.5 * rho * obj.v^2;                % 动压
        end
        
        % 计算过载
        function n = get_n(obj)
            R_B = obj.R_B();
            P_B = obj.P_B();
            Fa_B = obj.Fa_B();
            Fe_B = obj.Fe_B();
            n = (R_B(2)+P_B(2)+Fa_B(2)+Fe_B(2))/(obj.m*Earth.g_0);
        end
    end
    
    methods(Static)
        % 插值函数，平滑程序俯仰角
        function pitch_angle = interpolation(t, data)
            d2r = pi / 180;
            i = floor(t * 10) + 1;
            if 0 <= t && t < Rocket.t_stage(1) + Rocket.t_stage(2) + Rocket.t_stage(3)
                pitch_angle = (data(i,2) + (data(i+1,2)-data(i,2)) / (data(i+1,1) - data(i,1)) * (t - data(i,1))) * d2r;
            elseif t == Rocket.t_stage(1) + Rocket.t_stage(2) + Rocket.t_stage(3)
                pitch_angle = data(i,2) * d2r;
            else
                pitch_angle = 0;
            end
        end
    end
end
