classdef Plotter
    properties
        rocket;     % 火箭对象
        vec_idx;    % 各级关机点时间对应的索引
        h;          % 高度
        v;          % 速度
        theta_L;    % 地理经度
        Phi_L;      % 地理纬度
        m;          % 质量
        q;          % 动压
        n;          % 法向过载
        alpha;      % 攻角
        theta_v;    % 弹道倾角
        pitch;      % 俯仰角
        psi_v;      % 弹道偏角
    end
    
    methods
        %% 构造函数
        function obj = Plotter(rocket)
            obj.rocket = rocket;
            % 获取各级关机点时间对应的索引
            t_powered = obj.rocket.trajectory.t_powered;
            X_whole = obj.rocket.trajectory.X_whole;
            t_whole = obj.rocket.trajectory.t_whole;
            idx_stage1 = find(t_powered == obj.rocket.t_stage(1), 1);
            idx_stage2 = find(t_powered == obj.rocket.t_stage(1) + obj.rocket.t_stage(2), 1);
            idx_stage3 = find(t_powered == obj.rocket.t_stage(1) + obj.rocket.t_stage(2) + obj.rocket.t_stage(3), 1);
            obj.vec_idx = [idx_stage1, idx_stage2, idx_stage3];
            
            % 计算全弹道数据长度
            N_whole = size(t_whole, 1);
            obj.h = zeros(N_whole, 1);
            obj.v = zeros(N_whole, 1);
            obj.theta_L = zeros(N_whole, 1);
            obj.Phi_L = zeros(N_whole, 1);
            obj.m = zeros(N_whole, 1);
            obj.q = zeros(N_whole, 1);
            obj.n = zeros(N_whole, 1);
            obj.alpha = zeros(N_whole, 1);
            obj.theta_v = zeros(N_whole, 1);
            obj.pitch = zeros(N_whole, 1);
            obj.psi_v = zeros(N_whole, 1);
            % 计算火箭全弹道各参数数据
            for i = 1:size(t_whole,1)
                obj.rocket = obj.rocket.update(t_whole(i), X_whole(i,:));
                obj.pitch(i) = rad2deg(obj.rocket.pitch);
                obj.theta_v(i) = rad2deg(obj.rocket.theta_v);
                obj.psi_v(i) = rad2deg(obj.rocket.psi_v);
                obj.alpha(i) = rad2deg(obj.rocket.alpha);
                obj.h(i) = obj.rocket.h * 0.001;
                obj.v(i) = obj.rocket.v;
                obj.m(i) = obj.rocket.m;
                obj.q(i) = obj.rocket.q * 0.001;
                obj.theta_L(i) = rad2deg(obj.rocket.theta_L);
                obj.Phi_L(i) =  rad2deg(obj.rocket.Phi_L);
                obj.n(i) = obj.rocket.n;
            end
        end
        
        function obj = plot_trajectoryCurve(obj)
            X_powered = obj.rocket.trajectory.X_powered * 0.001;
            X_whole = obj.rocket.trajectory.X_whole * 0.001;
            
            %% 绘制主动段弹道曲线（发射坐标系下）
            figure (1);
            hold on
            plot3(X_powered(:,3),X_powered(:,1),X_powered(:,2));
            Plotter.plotShutdownPoint3(X_powered, obj.vec_idx);
            hold off
            view(3);
            axis equal;
            grid on;
            xlabel('z/km');
            ylabel('x/km');
            zlabel('y/km');
            title('发射坐标系下主动段弹道曲线');
            legend('主动段弹道曲线', '一级关机点', '二级关机点', '三级关机点');
            
            %% 绘制全弹道曲线（发射坐标系下）
            figure (2);
            hold on
            plot3(X_whole(:,3),X_whole(:,1),X_whole(:,2));
            Plotter.plotShutdownPoint3(X_whole, obj.vec_idx);
            hold off
            view(3);
            axis equal;
            grid on;
            xlabel('z/km');
            ylabel('x/km');
            zlabel('y/km');
            title('发射坐标系下全弹道曲线');
            legend('全弹道曲线', '一级关机点', '二级关机点', '三级关机点');
            
            %% 在地球上可视化弹道曲线（地心坐标系下）
            R = X_whole(:,1:3);
            % 所有时刻的地心坐标系下火箭地心矢量
            R_E = Rotation.L2E(obj.rocket.A_L0, obj.rocket.theta_T0, obj.rocket.Phi_T0) * R' + obj.rocket.R0_e*0.001;
            R_E = R_E';
            
            figure(3);
            hold on
            plot3(R_E(:,1), R_E(:,2), R_E(:,3), 'LineWidth', 3);
            for i = 1:length(obj.vec_idx)
                plot3(R_E(obj.vec_idx(i),1), R_E(obj.vec_idx(i),2), R_E(obj.vec_idx(i),3), '*');
            end
            ellipsoid(0, 0, 0, 0.001*Earth.a_e, 0.001*Earth.a_e, 0.001*Earth.b_e);
            hold off
            view(3);
            axis equal
            grid on
            xlabel('x/km');
            ylabel('y/km');
            zlabel('z/km');
            title('地心坐标系下弹道曲线');
            legend('弹道曲线', '一级关机点', '二级关机点', '三级关机点');
        end
        
        function obj = plot_poweredData(obj)
            t_powered = obj.rocket.trajectory.t_powered;
            idx_stage1 = find(t_powered == obj.rocket.t_stage(1), 1);
            %% 绘制一级飞行时最大攻角、最大动压和最大法向过载
            % 计算一级飞行最大攻角、最大动压和最大法向过载
            [max_alpha, idx_max_alpha] = min(obj.alpha(1:idx_stage1));
            [max_q, idx_max_q] = max(obj.q(1:idx_stage1));
            [max_n, idx_max_n] = max(obj.n(1:idx_stage1));
            
            % 打印到终端
            fprintf('一级飞行时最大攻角: %.2f° 在时间: %.2fs\n', max_alpha, t_powered(idx_max_alpha));
            fprintf('一级飞行时最大动压: %.2fkPa 在时间: %.2fs\n', max_q, t_powered(idx_max_q));
            fprintf('一级飞行时最大法向过载: %.2fg 在时间: %.2fs\n\n', max_n, t_powered(idx_max_n));
            
            figure(4);
            hold on
            plot(t_powered, obj.alpha(1: size(t_powered, 1)), 'r');    % 绘制攻角
            plot(t_powered, obj.pitch(1: size(t_powered, 1)), 'g');    % 绘制俯仰角
            plot(t_powered, obj.theta_v(1: size(t_powered, 1)), 'b');  % 绘制弹道倾角
            plot(t_powered(idx_max_alpha), max_alpha, 'o','MarkerFaceColor','k','HandleVisibility','off');
            text(t_powered(idx_max_alpha), max_alpha, sprintf('一级飞行时最大攻角:\n%.2f°', max_alpha));
            hold off
            xlabel('时间/s');
            ylabel('角度/°');
            title('攻角、俯仰角和弹道倾角随时间变化');
            legend('攻角', '俯仰角', '弹道倾角');  % 添加图例
            ylim([-40 90]);  % 设置y轴范围为0-90°
            grid on;
            
            %% 绘制主动段数据
            figure(5);
            subplot(3,3,1);
            hold on
            plot(t_powered, obj.h(1: size(t_powered, 1)));
            Plotter.plotShutdownPoint(t_powered, obj.h, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('高度/km');
            title('高度随时间变化');
            grid on;
            
            subplot(3,3,2);
            hold on
            plot(t_powered, obj.v(1: size(t_powered, 1)));
            Plotter.plotShutdownPoint(t_powered, obj.v, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('速度(m/s)');
            title('速度随时间变化');
            grid on;
            
            subplot(3,3,4);
            hold on
            plot(t_powered, obj.theta_L(1: size(t_powered, 1)));
            Plotter.plotShutdownPoint(t_powered, obj.theta_L, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('地理经度/°');
            title('地理经度随时间变化');
            grid on;
            
            subplot(3,3,5);
            hold on
            plot(t_powered, obj.Phi_L(1: size(t_powered, 1)));
            Plotter.plotShutdownPoint(t_powered, obj.Phi_L, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('地理纬度/°');
            title('地理纬度随时间变化');
            grid on;
            
            subplot(3,3,7);
            hold on
            plot(t_powered, obj.m(1: size(t_powered, 1)));
            Plotter.plotShutdownPoint(t_powered, obj.m, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('质量/kg');
            title('质量随时间变化');
            grid on;
            
            subplot(3,3,8);
            hold on
            plot(t_powered, obj.q(1: size(t_powered, 1)));
            plot(t_powered(idx_max_q), max_q, 'o','MarkerFaceColor','k','HandleVisibility','off');
            text(t_powered(idx_max_q), max_q, sprintf('一级飞行时最大动压:\n%.2fkPa', max_q));
            Plotter.plotShutdownPoint(t_powered, obj.q, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('动压/Pa');
            title('动压随时间变化');
            grid on;
            
            subplot(3,3,9);
            hold on
            plot(t_powered, obj.n(1: size(t_powered, 1)));
            plot(t_powered(idx_max_n), max_n, 'o','MarkerFaceColor','k','HandleVisibility','off');
            text(t_powered(idx_max_n), max_n, sprintf('一级飞行时最大法向过载:\n%.2fg', max_n));
            Plotter.plotShutdownPoint(t_powered, obj.n, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('过载/g');
            title('法向过载随时间变化');
            grid on;
            
            subplot(3,3,3);
            hold on
            plot(Inf,Inf, '*','Color','r');
            plot(Inf,Inf, '*','Color','g');
            plot(Inf,Inf, '*','Color','b');
            legend('一级关机点', '二级关机点', '三级关机点');
            hold off
            axis off;
        end
        function obj = plot_wholeData(obj)
            t_whole = obj.rocket.trajectory.t_whole;
            
            %% 绘制全弹道数据
            figure(6);
            subplot(3,3,1);
            hold on
            plot(t_whole, obj.h);
            Plotter.plotShutdownPoint(t_whole, obj.h, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('高度/km');
            title('高度随时间变化');
            grid on;
            
            subplot(3,3,2);
            hold on
            plot(t_whole, obj.v);
            Plotter.plotShutdownPoint(t_whole, obj.v, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('速度(m/s)');
            title('速度随时间变化');
            grid on;
            
            subplot(3,3,4);
            hold on
            plot(t_whole, obj.theta_L);
            Plotter.plotShutdownPoint(t_whole, obj.theta_L, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('地理经度/°');
            title('地理经度随时间变化');
            grid on;
            
            subplot(3,3,5);
            hold on
            plot(t_whole, obj.Phi_L);
            Plotter.plotShutdownPoint(t_whole, obj.Phi_L, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('地理纬度/°');
            title('地理纬度随时间变化');
            grid on;
            
            subplot(3,3,6);
            hold on
            plot(t_whole, obj.theta_v);
            Plotter.plotShutdownPoint(t_whole, obj.theta_v, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('弹道倾角/°');
            title('弹道倾角随时间变化');
            grid on;
            
            % subplot(3,3,6);
            % hold on
            % plot(t_whole, obj.psi_v);
            % Plotter.plotShutdownPoint(t_whole, obj.psi_v, obj.vec_idx);
            % hold off
            % xlabel('时间/s');
            % ylabel('弹道偏角/°');
            % title('弹道倾角随时间变化');
            % grid on;
            
            subplot(3,3,7);
            hold on
            plot(t_whole, obj.m);
            Plotter.plotShutdownPoint(t_whole, obj.m, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('质量/kg');
            title('质量随时间变化');
            grid on;
            
            subplot(3,3,8);
            hold on
            plot(t_whole, obj.q);
            Plotter.plotShutdownPoint(t_whole, obj.q, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('动压/Pa');
            title('动压随时间变化');
            grid on;
            
            subplot(3,3,9);
            hold on
            plot(t_whole, obj.n);
            Plotter.plotShutdownPoint(t_whole, obj.n, obj.vec_idx);
            hold off
            xlabel('时间/s');
            ylabel('过载/g');
            title('法向过载随时间变化');
            grid on;
            
            subplot(3,3,3);
            hold on
            plot(Inf,Inf, '*','Color','r');
            plot(Inf,Inf, '*','Color','g');
            plot(Inf,Inf, '*','Color','b');
            legend('一级关机点', '二级关机点', '三级关机点');
            hold off
            axis off;
        end
    end
    methods(Static)
        function plotShutdownPoint(t, data, indexs)
            plot(t(indexs(1)), data(indexs(1)), '*', 'Color', 'r');
            plot(t(indexs(2)), data(indexs(2)), '*', 'Color', 'g');
            plot(t(indexs(3)), data(indexs(3)), '*', 'Color', 'b');
        end
        
        function plotShutdownPoint3(X, indexs)
            plot3(X(indexs(1),3), X(indexs(1),1), X(indexs(1),2), '*', 'Color', 'r');
            plot3(X(indexs(2),3), X(indexs(2),1), X(indexs(2),2), '*', 'Color', 'g');
            plot3(X(indexs(3),3), X(indexs(3),1), X(indexs(3),2), '*', 'Color', 'b');
        end
    end
end
