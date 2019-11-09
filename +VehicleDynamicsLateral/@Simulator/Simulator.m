classdef Simulator<handle
    % Simulator Vehicle dynamics simulator
    % The simulator receives a vehicle object that inherits from VehicleSimple, simulates its behavior during a given time span and provides its behavior during time via its properties. Each property is a (timespan, 1) vector in which each value represents that parameter's value in time.
    methods
        % Constructor
        function self = Simulator(vehicle, tspan)
            self.Vehicle = vehicle;
            self.TSpan = tspan;
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                self.X0 = 0;
                self.Y0 = 0;
                self.PSI0 = 0;
                self.PHI0 = 0;
                self.V0 = 20;
                self.ALPHAT0 = 0;
                self.dPSI0 = 0;
                self.dPHI0 = 0;
            else
                if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleSimpleNonlinear4DOF')
                    self.X0 = 0;
                    self.Y0 = 0;
                    self.PSI0 = 0;
                    self.THETA0 = 0;
                    self.V0 = 20;
                    self.ALPHAT0 = 0;
                    self.dPSI0 = 0;
                    self.dTHETA0 = 0;
                else
                    self.X0 = 0;
                    self.Y0 = 0;
                    self.PSI0 = 0;
                    self.V0 = 20;
                    self.ALPHAT0 = 0;
                    self.dPSI0 = 0;
                end
            end
        end

        function f = getInitialState(self)
            % Transforms properties into a vector so it can be used by the integrator
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                f = [self.X0 self.Y0 self.PSI0 self.PHI0 self.V0 self.ALPHAT0 self.dPSI0 self.dPHI0];
            else
                if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleSimpleNonlinear4DOF')
                    f = [self.X0 self.Y0 self.PSI0 self.THETA0 self.V0 self.ALPHAT0 self.dPSI0 self.dTHETA0];
                else
                    f = [self.X0 self.Y0 self.PSI0 self.V0 self.ALPHAT0 self.dPSI0];
                end
            end


        end

        function Simulate(self)
            % TODO: gravity can be passed to the simulator so vertical load and other forces are calculated here

            % integration
            % if vehicle is articulated, adds mass matrix as an integration option
            if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleArticulated')
                fun = self.Vehicle;
                options = odeset('Mass', @fun.MassMatrix,'Events', @fun.velocity);
                [TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados,self.TSpan), self.TSpan, self.getInitialState(), options);
                % retrieve states exclusive to the articulated vehicle
                self.XT = XOUT(:, 1);
                self.YT = XOUT(:, 2);
                self.PSI = XOUT(:, 3);
                self.PHI = XOUT(:, 4);
                self.VEL = XOUT(:, 5);
                self.ALPHAT = XOUT(:, 6);
                self.dPSI = XOUT(:, 7);
                self.dPHI = XOUT(:, 8);
            else
                if isa(self.Vehicle, 'VehicleDynamicsLateral.VehicleSimpleNonlinear4DOF')
                    fun = self.Vehicle;
                    options = odeset('Mass', @fun.MassMatrix);
                    [TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados,self.TSpan), self.TSpan, self.getInitialState(), options);

                    % Retrieving states post integration
                    self.XT = XOUT(:, 1);
                    self.YT = XOUT(:, 2);
                    self.PSI = XOUT(:, 3);
                    self.THETA = XOUT(:, 4);
                    self.VEL = XOUT(:, 5);
                    self.ALPHAT = XOUT(:, 6);
                    self.dPSI = XOUT(:, 7);
                    self.dTHETA = XOUT(:, 8);
                else
                    [TOUT, XOUT] = ode45(@(t, estados) self.Vehicle.Model(t, estados,self.TSpan), self.TSpan, self.getInitialState());

                    % Retrieving states post integration
                    self.XT = XOUT(:, 1);
                    self.YT = XOUT(:, 2);
                    self.PSI = XOUT(:, 3);
                    self.VEL = XOUT(:, 4);
                    self.ALPHAT = XOUT(:, 5);
                    self.dPSI = XOUT(:, 6);
                end
            end

            % TSpan and TOUT contain the same values, but the first is passed in columns, while the second is a vector
            self.TSpan = TOUT;
        end
    end

    properties
        Vehicle % Vehicle model to be used inthe simulation
        TSpan % a vector indicating the intervals in which the simulation steps will be conducted
        X0 % Initial CG horizontal position [m]
        Y0 % Initial CG vertical position [m]
        PSI0 % Initial yaw angle [rad]
        PHI0 % Initial articulation angle [rad]
        THETA0 % Initial roll angle [rad]
        V0 % Initial CG velocity [m/s]
        ALPHAT0 % Initial side slip angle [rad]
        dPSI0 % Initial yaw rate [rad/s]
        dPHI0 % Initial articulation rate [rad/s]
        dTHETA0 % Initial roll rate [rad/s]
        XT % CG horizontal position [m]
        YT % CG vertical position [m]
        PSI % Yaw angle [rad]
        PHI % Relative yaw angle [rad]
        THETA % Roll angle [rad]
        VEL % CG velocity [m/s]
        ALPHAT % Side slip angle [rad]
        dPSI % Yaw rate [rad/s]
        dPHI % Relative yaw rate [rad/s]
        dTHETA % Roll rate [rad/s]
    end
end

%% See Also
%
% <../../index.html Home>
%
