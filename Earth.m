classdef Earth
    properties(Constant)
        a_e = 6371004;      % 地球赤道半径
        b_e = 6356754;      % 地球极半径
        J2 = 1.08263e-3;    % 地球二阶带谐常数
        e_E = 1/298.257;    % 地球扁率
        mu = 3.986e+14;     % 地球引力常量
        g_0 = 9.8066;       % 地表重力加速度
        omega = 7.292115e-5;% 地球自转角速度
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
    end
end
