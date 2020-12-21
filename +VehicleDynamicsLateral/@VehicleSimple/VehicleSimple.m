classdef (Abstract) VehicleSimple
    % VehicleSimple Simple vehicle abstract class.
    %
    % Abstract class representing a simple vehicle.
    %
    % Extend this class in order to create a new vehicle model to be used with the simulator.

    methods(Abstract)
        Model(self, t, estados)
    end

    properties
        mT     % Mass of the car (tractor) [kg]
        IT     % Moment of inertia the car (tractor) [kg * m2]
        a      % Distance from front axle of the car (tractor) to the center of mass of the car (tractor) [m]
        b      % Distance from center of mass of the car (tractor) to the front axle of the car (tractor) [m]
        mF0    % Mass over the front axle [kg]
        mR0    % Mass over the rear axle [kg]
        lT     % Wheelbase [m]
        nF     % Number of front tires
        nR     % Number of rear tires
        wT     % Track of the car (tractor)  [m]
        muy    % Operational friction coefficient
        tire   % Tire model
        deltaf % Steering angle [rad]
        deltar % Steering angle [rad]
        Fxf    % Longitudinal force at F [rad]
        Fxr    % Longitudinal force at R [rad]
    end

    methods

        function value = get.mT(self)
            value = self.mF0 + self.mR0;
        end

        function value = get.a(self)
            value = self.mR0 / self.mT * self.lT;
        end

        function value = get.b(self)
            value = self.lT - self.a;
        end
    end

end

%% See Also
%
% <../../index.html Home>
%
