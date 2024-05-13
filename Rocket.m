classdef Rocket
    properties(Constant)%% 常量
        %% 火箭参数
        m = [19000 5400 1300];      % 各级火箭质量
        d = [1.0 1.0 1.0];          % 各级火箭直径
        P = [480000 120000 40000];   % 各级火箭推力
        dm = [170 45 15];           % 各级发动机秒耗量
        t = [70 75 60];             % 各级工作时间
        
        %% 气动参数
        Cx = 0.1;   % 阻力系数
        Cy = 0.15;   % 升力系数
        Cz = 0;     % 侧力系数
    end
    properties%% 状态变量
        %% 火箭状态
        X;                  % 火箭状态量
        R_launch;           % 火箭位置（发射坐标系下）
        V_launch;           % 火箭速度（发射坐标系下）
        
        pitch;              % 火箭俯仰角
        yaw;                % 火箭偏航角
        roll;               % 火箭滚转角
        
        r;                  % 火箭地心距离
        h;                  % 火箭海拔高度
        v;                  % 火箭速度大小
        A_L;                % 火箭地理方位角
        theta_L;            % 火箭地理经度
        Phi_L;              % 火箭地理纬度
        theta_T;            % 火箭天文经度
        Phi_T;              % 火箭天文纬度
        
    end
    properties
        %% 发射点参数
        A_L0;               % 发射点地理方位角
        theta_T0;           % 发射点地理经度
        Phi_T0;             % 发射点地理纬度
        
        r0;                 % 发射点地心距离
        R0_e;               % 发射点地心矢径(地心坐标系下)
        R0_L;               % 发射点地心矢径（发射坐标系下）
        
        Rc_e;               % 火箭地心矢径(地心坐标系下)
        Rc_L;               % 火箭地心矢径（发射坐标系下）
        r_Ue;               % 火箭星下点地心距
    end
    
    methods
        %% 构造函数
        function obj = Rocket(A_L0, theta_L0, Phi_L0)
            % 发射点参数,将发射点的地理经纬度转换为天文经纬度
            obj.A_L0 = A_L0;
            obj.theta_T0 = Earth.theta_L2T(theta_L0);
            obj.Phi_T0 = Earth.Phi_L2T(Phi_L0);
            disp('发射点参数:')
            fprintf('发射方位角: %.2f°  地理经度: %.2f°  地理纬度: %.2f°\n\n', Earth.rad2deg(A_L0), Earth.rad2deg(theta_L0), Earth.rad2deg(Phi_L0));
            % 发射点地心距离
            obj.r0 = Earth.a_e * (1 - Earth.e_E)/sqrt((sin(Phi_L0))^2 + (1-Earth.e_E)^2 * (cos(Phi_L0))^2);
            obj.R0_e = obj.r0 * [cos(Phi_L0) * cos(theta_L0);cos(Phi_L0) * sin(theta_L0);sin(Phi_L0)];
            obj.R0_L = Rotation.L2E(A_L0, theta_L0, Phi_L0)' * obj.R0_e;
            % 火箭状态量初始化
            obj.X = [0; 0; 0; 0; 0; 0; Rocket.m(1)];
            obj.A_L = A_L0;
            obj = obj.update(obj.X);
        end
        
        %% 更新状态(根据变化后的X更新状态量)
        function obj = update(obj, X)
            obj.X = X(:); % 将 X 转换为列向量
            obj.R_launch = obj.X(1:3);
            obj.V_launch = obj.X(4:6);
            
            obj.Rc_e = obj.get_Rc_e();
            obj.Rc_L = obj.get_Rc_L();
            obj.r = norm(obj.Rc_e);
            
            obj.v = norm(obj.V_launch);
            % todo 计算地理方位角
            % obj.A_L = atan2(obj.R_launch(2), obj.R_launch(1));
            obj.theta_L = atan2(obj.Rc_e(2), obj.Rc_e(1));
            obj.Phi_L = asin(obj.Rc_e(3) / obj.r);
            obj.theta_T = Earth.theta_L2T(obj.theta_L);
            obj.Phi_T = Earth.Phi_L2T(obj.Phi_L);
            
            obj.r_Ue = Earth.a_e * (1 - Earth.e_E)/sqrt((sin(obj.Phi_T))^2 + (1-Earth.e_E)^2 * (cos(obj.Phi_T))^2);
            obj.h = obj.r - obj.r_Ue;
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
        
        % 火箭受到的气动力（发射坐标系下）
        function R_L = R_L(obj)
            % todo气动力在速度坐标系下的分量
            R = Rocket.R_v(Rocket.d(1), obj.h, obj.v);
            
            % 计算弹道倾角和弹道偏角
            if obj.V_launch(2) < 1 || obj.V_launch(1) < 0.0001
                path_angle = pi/2;
                defl_angle = 0;
            else
                path_angle = atan(obj.V_launch(2) / obj.V_launch(1));
                defl_angle = atan(-obj.V_launch(3) / (cos(path_angle) * obj.V_launch(1) + sin(path_angle) * (obj.V_launch(2) + 0.00000001)));
            end
            
            slant_angle = 0;
            
            % 气动力在发射坐标系下的分量
            R_L = Rotation.L2V(slant_angle, defl_angle, path_angle)' * R;
        end
        
        % 火箭受到的推力（发射坐标系下）
        function P_L = P_L(~, t, data)
            if t < Rocket.t(1)
                [pitch_angle,yaw_angle,roll_angle] = Rocket.interpolation(t, data);
                P_L = Rotation.L2B(pitch_angle,yaw_angle,roll_angle)' * [Rocket.P(1); 0; 0];
            elseif t < Rocket.t(1) + Rocket.t(2)
                [pitch_angle,yaw_angle,roll_angle] = Rocket.interpolation(t, data);
                P_L = Rotation.L2B(pitch_angle,yaw_angle,roll_angle)' * [Rocket.P(2); 0; 0];
            elseif t < Rocket.t(1) + Rocket.t(2) + Rocket.t(3)
                [pitch_angle,yaw_angle,roll_angle] = Rocket.interpolation(t, data);
                P_L = Rotation.L2B(pitch_angle,yaw_angle,roll_angle)' * [Rocket.P(3); 0; 0];
            else
                P_L = [0; 0; 0];    %被动段无推力
            end
        end
        
        % 火箭受到的科氏惯性力（发射坐标系下）
        function Fa_L = Fa_L(obj)
            % 地球自转角速度矢量（发射坐标系下）
            omega_L = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0)' * [0; 0; Earth.omega];
            Fa_L = -2 * obj.X(7) * cross(omega_L, obj.V_launch);
        end
        
        % 火箭受到的牵连惯性力（发射坐标系下）
        function Fe_L = Fe_L(obj)
            % 地球自转角速度矢量（发射坐标系下）
            omega_L = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0)' * [0; 0; Earth.omega];
            Fe_L = -obj.X(7) * cross(omega_L, cross(omega_L, obj.V_launch));
        end
    end
    
    methods(Access = private)
        % 火箭地心矢径（地心坐标系下）
        function Rc_e = get_Rc_e(obj)
            Rc_e = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0) * obj.R_launch + obj.R0_e;
            
            % 另一种写法，与上面等价
            % Rc_e = Rotation.L2E(obj.A_L0, obj.theta_T0, obj.Phi_T0) * obj.get_Rc_L();
        end
        
        % 火箭地心矢径(发射坐标系下)
        function Rc_L = get_Rc_L(obj)
            Rc_L = obj.R_launch + obj.R0_L;
        end
    end
    
    methods(Static)
        % 火箭受到的气动力（速度坐标系下）
        function R_v = R_v(d, h, v)
            rho = 1.225 * exp(-1.406e-4 * h);   % 大气密度
            q = 0.5 * rho * v^2;                % 动压
            S = pi * (d/2)^2;                   % 特征面积
            
            D = Rocket.Cx * q * S;
            L = Rocket.Cy * q * S;
            Z = Rocket.Cz * q * S;
            
            R_v = [-D; L; Z];
        end
        function [pitch_angle,yaw_angle,roll_angle] = interpolation(t, data)
            d2r = pi / 180;
            i = floor(t * 10) + 1;
            
            pitch_angle = (data(i,2) + (data(i+1,2)-data(i,2)) / (data(i+1,1) - data(i,1)) * (t - data(i,1))) * d2r;
            yaw_angle = 0;
            roll_angle = 0;
        end
    end
end
