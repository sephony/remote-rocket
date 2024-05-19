classdef Earth
    properties(Constant)
        % 地球赤道半径
        a_e = 6371004;
        % 地球极半径
        b_e = 6356754;
        % 地球二阶带谐常数
        J2 = 1.08263e-3;
        % 地球扁率
        e_E = 1/298.257;
        % 地球引力常量
        mu = 3.986e+14;
        % 地表重力加速度
        g_0 = 9.8066;
        % 地球自转角速度
        omega = 7.292115e-5;
    end
    methods(Static)
        % 地理经度转天文经度
        function theta_T = theta_L2T(lati_G)
            theta_T = lati_G;
        end
        % 地理纬度转天文纬度
        function Phi_T = Phi_L2T(lati_G)
            Phi_T = lati_G + Earth.e_E * sin(2 * lati_G);
        end
        % 角度转弧度
        function rad = deg2rad(deg)
            rad = deg * pi / 180;
        end
        % 弧度转角度
        function deg = rad2deg(rad)
            deg = rad * 180 / pi;
        end
    end
end
