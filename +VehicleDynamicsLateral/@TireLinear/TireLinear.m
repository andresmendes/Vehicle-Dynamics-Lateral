classdef TireLinear < VehicleDynamicsLateral.Tire
    % TireLinear Linear tire model
    %
    % It inherits methods from Tire.

    methods
        % Constructor
        function self = TireLinear()
            self.k = 40000;
        end

        function p = PlotTire(self)
            % Returns the handle of the curve
            alpha = (0:0.1:15)*pi/180;
            Fy = - self.Characteristic(alpha);
            p = plot(alpha*180/pi,Fy);
            grid on; box on;
            xlabel('Slip angle [deg]')
            ylabel('Lateral force [N]')
        end

        function Fy = Characteristic(self, alpha, varargin)
            Fy = - self.k * alpha;
        end
    end

    properties
        k % Cornering stiffness [N/rad]
    end

end

%% See Also
%
% <../../index.html Home>
%
