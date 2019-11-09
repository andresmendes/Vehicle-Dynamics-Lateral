classdef TirePolynomial < VehicleDynamicsLateral.Tire
    % TirePolynomial Polynomial tire model
    %
    % It inherits methods from Tire.

    methods
        % Constructor
        function self = TirePolynomial()
            self.k1 = 115000;
            self.k2 = 560000;
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
            % Lateral force
            Fy = - (self.k1 * alpha - self.k2 * alpha.^3);
        end
    end

    properties
        k1 % 1st polynomial coefficient, cornering stiffness [N/rad]
        k2 % 2nd polynomial coefficient [N/rad^3]
    end
end

%% See Also
%
% <../../index.html Home>
%
