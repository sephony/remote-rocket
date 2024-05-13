classdef Rotation
    methods(Access = public, Static)
        function C = x(angle)
            C = [1, 0, 0;
                0, cos(angle), sin(angle);
                0, -sin(angle), cos(angle)];
        end
        
        function C = y(angle)
            C = [cos(angle), 0, -sin(angle);
                0, 1, 0;
                sin(angle), 0, cos(angle)];
        end
        
        function C = z(angle)
            C = [cos(angle), sin(angle), 0;
                -sin(angle), cos(angle), 0;
                0, 0, 1];
        end
        % A_L:发射点方位角; theta_T:天文经度; Phi_T:天文纬度
        function C = L2E(A_L, theta_T, Phi_T)
            C = Rotation.z(pi / 2 - theta_T) * Rotation.x(-Phi_T) * Rotation.y(pi / 2 + A_L);
        end
        
        % pitch:俯仰角; yaw:偏航角; roll:滚转角
        function C = L2B(pitch, yaw, roll)
            C = Rotation.x(roll) * Rotation.y(yaw) * Rotation.z(pitch);
        end
        
        %path:弹道倾角; slant:倾侧角; defl:弹道偏角
        function C = L2V(slant, defl, path)
            C = Rotation.x(slant) * Rotation.y(defl) * Rotation.z(path);
        end
    end
end
