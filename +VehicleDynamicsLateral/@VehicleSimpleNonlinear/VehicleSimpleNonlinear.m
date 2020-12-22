classdef VehicleSimpleNonlinear < VehicleDynamicsLateral.VehicleSimple
    % VehicleSimpleNonlinear Nonlinear simple vehicle model.
    %
    % It inherits properties from VehicleSimple.

    methods
        % Constructor
        function self = VehicleSimpleNonlinear()
            self.mF0 = 700;
            self.mR0 = 600;
            self.IT = 10000;
            self.lT = 3.5;
            self.nF = 2;
            self.nR = 2;
            self.wT = 2;
            self.muy = .8;
            self.deltaf = 0;
            self.deltar = 0;
            self.Fxf = 0;
            self.Fxr = 0;
        end

        function dx = Model(self, t, states,tspan)
            % dx = VehicleModel.Model(t,states,tspan)
            % dx = VehicleModel.MassMatrix(t,states,tspan)
            %
            % Sintax
            % |dx = _VehicleModel_.Model(t,states,tspan)|
            %
            % Arguments
            % The following table describes the input arguments:
            %
            % t - Time
            % states - Model state variables: [XT YT PSI VT ALPHAT dPSI]
            % tspan - Time span
            
            % Data
            mT  = self.mT;
            IT  = self.IT;
            a   = self.a;
            b   = self.b;
            nF  = self.nF;
            nR  = self.nR;
            muy = self.muy;

            g = 9.81;                 % Gravity [m/s^2]

            FzF = self.mF0 * g;       % Vertical load @ F [N]
            FzR = self.mR0 * g;       % Vertical load @ R [N]

            % Estados
            X       = states(1);
            Y       = states(2);
            PSI     = states(3);
            VT      = states(4);
            ALPHAT  = states(5);
            dPSI    = states(6);

            % Steering input
            if isa(self.deltaf,'function_handle')
                deltaf = self.deltaf([X;Y;PSI;VT;ALPHAT;dPSI],t);
            elseif length(self.deltaf)>1
                deltaf = interp1(tspan,self.deltaf,t);
            else
                deltaf = self.deltaf;
            end
            
            if isa(self.deltar,'function_handle')
                deltar = self.deltar([X;Y;PSI;VT;ALPHAT;dPSI],t);
            elseif length(self.deltar)>1
                deltar = interp1(tspan,self.deltar,t);
            else
                deltar = self.deltar;
            end

            % Slip angles
            ALPHAF = - deltaf + atan2(a*dPSI + VT*sin(ALPHAT), VT*cos(ALPHAT)); % Dianteiro
            ALPHAR = - deltar + atan2(VT*sin(ALPHAT) - b*dPSI, VT*cos(ALPHAT));         % Traseiro

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

            % Characteristic curve
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF/nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR/nR, muy);

            % Equations of motion
            dx(1,1) = VT * cos(ALPHAT + PSI);
            dx(2,1) = VT*sin(ALPHAT + PSI);
            dx(3,1) = dPSI;
            dx(4,1) = (FxF*cos(ALPHAT - deltaf) + FxR*cos(ALPHAT - deltar) + FyF*sin(ALPHAT - deltaf) + FyR*sin(ALPHAT - deltar))/mT;
            dx(5,1) = -(FxF*sin(ALPHAT - deltaf) - FyR*cos(ALPHAT - deltar) - FyF*cos(ALPHAT - deltaf) + FxR*sin(ALPHAT - deltar) + VT*dPSI*mT)/(VT*mT);
            dx(6,1) = (FyF*a*cos(deltaf) - FyR*b*cos(deltar) + FxF*a*sin(deltaf) - FxR*b*sin(deltar))/IT;

        end
    end
end

%% See Also
%
% <../../index.html Home>
%
