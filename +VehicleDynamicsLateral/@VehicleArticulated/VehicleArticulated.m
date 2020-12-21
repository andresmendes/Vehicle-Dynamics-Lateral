classdef (Abstract) VehicleArticulated  < VehicleDynamicsLateral.VehicleSimple
    % VehicleArticulated Articulated vehicle abstract class.
    %
    % Abstract class representing an articulated vehicle. It inherits properties from VehicleSimple and has additional properties related to the semitrailer.
    %
    % Extend this class in order to create a new vehicle model to be used with the simulator.

    methods(Abstract)
        % MassMatrix(self, t, estados)
    end

    properties
        mF  % Mass over the front axle when the semitrailer is coupled [kg]
        mR  % Mass over the rear axle when the semitrailer is coupled [kg]
        mM  % Mass over the semitrailer axle when the semitrailer is coupled [kg]
        mA  % Mass over the articulation point when the semitrailer is coupled [kg]
        mS  % Mass of the semitrailer [kg]
        IS  % Moment of inertia of the semitrailer [kg.m2]
        nM  % Number of tires in semitrailer axle
        wS  % Track of the semitrailer [m]
        lS  % Distance from articulation to semitrailer axle (A-M) [m]
        c   % Distance from rear axle of the tractor to articulation (R-A) [m]
        d   % Distance from articulation to semitrailer center of mass (A-M) [m]
        e   % Distance from semitrailer center of mass to semitrailer axle (A-M) [m]
        Fxm % Longitudinal force at M [rad]
        deltam 
    end

    methods

        function value = get.mA(self)
            value = self.mF + self.mR - self.mT;
        end

        function value = get.mS(self)
            value = self.mA + self.mM;
        end

        function value = get.d(self)
            value = (self.lS * self.mM) / self.mS;
        end

        function value = get.e(self)
            value = self.lS - self.d;
        end
    end
end

%% See Also
%
% <../../index.html Home>
%
