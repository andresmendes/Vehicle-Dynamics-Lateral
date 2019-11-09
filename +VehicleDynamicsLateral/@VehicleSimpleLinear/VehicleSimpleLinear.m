classdef VehicleSimpleLinear < VehicleDynamicsLateral.VehicleSimple
    % VehicleSimpleLinear Linear simple vehicle model.
    %
    % It inherits properties from VehicleSimple.

    methods
        function self = VehicleSimpleLinear()
            % Constructor for the vehicle
            self.mF0 = 700;
            self.mR0 = 600;
            self.IT = 10000;
            self.lT = 3.5;
            self.nF = 2;
            self.nR = 2;
            self.wT = 2;
            self.muy = .8;
            self.deltaf = 0;
            self.Fxf = 0;
            self.Fxr = 0;
        end

        %% Model
        % Function with the model
        function dx = Model(self, t, states,tspan)
            % Data
            mT = self.mT;
            IT = self.IT;
            a = self.a;
            b = self.b;
            nF = self.nF;
            nR = self.nR;
            muy = self.muy;



            g = 9.81;                 % Gravity [m/s^2]

            FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

            v0 = 20;                  % [m/s]

            % State variables
            X = states(1,1);         % Not used
            Y = states(2,1);         % Not used
            PSI     = states(3,1);
            VT       = states(4,1);
            ALPHAT  = states(5,1);
            dPSI    = states(6,1);

            if isa(self.deltaf,'function_handle')
                deltaf = self.deltaf([X;Y;PSI;VT;ALPHAT;dPSI],t);
            elseif length(self.deltaf)>1
                deltaf = interp1(tspan,self.deltaf,t);
            else
                deltaf = self.deltaf;
            end

            % Slip angles
            ALPHAF = ALPHAT + a/v0*dPSI - deltaf;
            ALPHAR = ALPHAT - b/v0*dPSI;

            % Longitudinal forces
            if isa(self.Fxf,'function_handle')
                FxF = self.Fxf([X;Y;PSI;VT;ALPHAT;dPSI],t);
            elseif length(self.Fxf)>1
                FxF = interp1(tspan,self.Fxf,t);
            else
                FxF = self.Fxf;
            end

            if isa(self.Fxr,'function_handle')
                FxR = self.Fxr([X;Y;PSI;VT;ALPHAT;dPSI],t);
            elseif length(self.Fxr)>1
                FxR = interp1(tspan,self.Fxr,t);
            else
                FxR = self.Fxr;
            end

            % Lateral force
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF / nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR / nR, muy);

            % State equations
            dx(1,1) = VT;
            dx(2,1) = v0*(PSI + ALPHAT);
            dx(3,1) = dPSI;
            dx(4,1) = (FxF + FxR)/mT;
            dx(5,1) = (FyF + FyR)/(mT*v0) - dPSI;
            dx(6,1) = (a*FyF - b*FyR)/IT;

        end
    end
end

%% See Also
%
% <../../index.html Home>
%
