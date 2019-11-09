classdef VehicleArticulatedNonlinear < VehicleDynamicsLateral.VehicleArticulated
    % VehicleArticulatedNonlinear Nonlinear articulated vehicle model.
    %
    % It inherits properties from VehicleArticulated.

    methods
        % Constructor
        function self = VehicleArticulatedNonlinear()
            self.mF0 = 5200;
            self.mR0 = 2400;
            self.mF = 6000;
            self.mR = 10000;
            self.mM = 17000;
            self.IT = 46000;
            self.IS = 450000;
            self.lT = 3.5;
            self.lS = 7.7;
            self.c = -0.3;
            self.nF = 2;
            self.nR = 4;
            self.nM = 8;
            self.wT = 2.6;
            self.wS = 2.4;
            self.muy = 0.3;
            self.deltaf = 0;
            self.Fxf = 0;
            self.Fxr = 0;
            self.Fxm = 0;
        end

        function dx = Model(self,t, states,tspan)
            % Vehicle parameters
            mT = self.mT;
            mS = self.mS;
            % IT = self.IT;
            % IS = self.IS;
            a = self.a;
            b = self.b;
            c = self.c;
            d = self.d;
            e = self.e;
            nF = self.nF;
            nR = self.nR;
            nM = self.nM;

            g = 9.81;

            FzF = self.mF * g;
            FzR = self.mR * g;
            FzM = self.mM * g;
            muy = self.muy;


            % States
            X = states(1,1);
            Y = states(2,1);
            PSI = states(3,1);
            PHI = states(4,1);
            VT = states(5,1);
            ALPHAT = states(6,1);
            dPSI = states(7,1);
            dPHI = states(8,1);

            if isa(self.deltaf,'function_handle')
                deltaf = self.deltaf([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.deltaf)>1
                deltaf = interp1(tspan,self.deltaf,t);
            else
                deltaf = self.deltaf;
            end

            % Slip angles
            ALPHAF = atan2((a * dPSI + VT * sin(ALPHAT)),(VT * cos(ALPHAT))) - deltaf;
            ALPHAR = atan2((-b * dPSI + VT * sin(ALPHAT)),(VT * cos(ALPHAT)));
            ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VT * sin(ALPHAT + PHI) - b * dPSI * cos(PHI) - ...
                     c * dPSI * cos(PHI)),(VT * cos(ALPHAT + PHI) + b * dPSI * sin(PHI) + c * dPSI * sin(PHI)));

            % Longitudinal forces
            if isa(self.Fxf,'function_handle')
                FxF = self.Fxf([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.Fxf)>1
                FxF = interp1(tspan,self.Fxf,t);
            else
                FxF = self.Fxf;
            end

            if isa(self.Fxr,'function_handle')
                FxR = self.Fxr([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.Fxr)>1
                FxR = interp1(tspan,self.Fxr,t);
            else
                FxR = self.Fxr;
            end

            if isa(self.Fxm,'function_handle')
                FxM = self.Fxm([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.Fxm)>1
                FxM = interp1(tspan,self.Fxm,t);
            else
                FxM = self.Fxm;
            end

            % Lateral forces
            FyF = nF * self.tire.Characteristic(ALPHAF, FzF/nF, muy);
            FyR = nR * self.tire.Characteristic(ALPHAR, FzR/nR, muy);
            FyM = nM * self.tire.Characteristic(ALPHAM, FzM/nM, muy);

            f = [...
            VT*cos(PSI+ALPHAT);...
            VT*sin(PSI+ALPHAT);...
            dPSI;...
            dPHI;...
            FxF*cos(PSI + deltaf) + FxR*cos(PSI) + FxM*cos(PSI - PHI) - FyF*sin(PSI + deltaf) - FyR*sin(PSI) - FyM*sin(PSI - PHI) - mS*(b+c)*dPSI^2*cos(PSI) - mS*d*(dPSI - dPHI)^2*cos(PSI - PHI) + (mT + mS)*VT*sin(PSI+ALPHAT)*dPSI;...
            FxF*sin(PSI + deltaf) + FxR*sin(PSI) + FxM*sin(PSI - PHI) + FyF*cos(PSI + deltaf) + FyR*cos(PSI) + FyM*cos(PSI - PHI) - mS*(b+c)*dPSI^2*sin(PSI) - mS*d*(dPSI - dPHI)^2*sin(PSI - PHI) - (mT + mS)*VT*cos(PSI+ALPHAT)*dPSI;...
            FxF*a*sin(deltaf) + FxM*(b + c)*sin(PHI) + FyF*a*cos(deltaf) - FyR*b - FyM*((b+c)*cos(PHI) + (d+e)) - mS*(b+c)*d*(dPSI - dPHI)^2*sin(PHI) + mS*(b+c)*d*dPSI^2*sin(PHI) + mS*((b+c)*VT*cos(ALPHAT) + d*VT*cos(ALPHAT + PHI))*dPSI;...
            FyM*(d + e) - mS*(b+c)*d*dPSI^2*sin(PHI) - mS*d*VT*cos(ALPHAT + PHI)*dPSI ];

            dx = f;
        end

        function [value,isterminal,direction] = velocity(~,~,states)
            % If the velocity is less than 0.1m/s the integrator stops.
            % The MassMatrix is singular when the velocity is 0 m/s.
            value = states(5,1) - 0.1;
            isterminal = 1;
            direction = -1;
        end

        function M = MassMatrix(self,~,states)
            % Vehicle Parameters
            mT = self.mT;
            mS = self.mS;
            IT = self.IT;
            IS = self.IS;
            % a = self.a;
            b = self.b;
            c = self.c;
            d = self.d;
            % e = self.e;
            % deltaf = self.deltaf;
            % nF = self.nF;
            % nR = self.nR;
            % nM = self.nM;

            % g = 9.81;

            % FzF = self.mF * g;
            % FzR = self.mR * g;
            % FzM = self.mM * g;
            % muy = self.muy;


            % States
            PSI = states(3,1);
            PHI = states(4,1);
            VT = states(5,1);
            ALPHAT = states(6,1);
            % dPSI = states(7,1);
            % dPHI = states(8,1);

            % Matriz de massa
            M55 = (mT + mS)*cos(PSI + ALPHAT);
            M56 = -(mT + mS)*VT*sin(PSI + ALPHAT);
            M57 = mS*( (b+c)*sin(PSI) + d*sin(PSI - PHI) );
            M58 = -mS*d*sin(PSI - PHI);
            M65 = (mT + mS)*sin(PSI + ALPHAT);
            M66 = (mT + mS)*VT*cos(PSI + ALPHAT);
            M67 = -mS*( (b+c)*cos(PSI) + d*cos(PSI - PHI) );
            M68 = mS*d*cos(PSI - PHI);
            M75 = -mS*( (b+c)*sin(ALPHAT) + d*sin(ALPHAT + PHI) );
            M76 = -mS*( (b+c)*VT*cos(ALPHAT) + d*VT*cos(ALPHAT + PHI) );
            M77 = mS*( (b+c)^2 + 2*(b+c)*d*cos(PHI) + d^2 ) + IT + IS;
            M78 = -( mS*( (b+c)*d*cos(PHI) + d^2 ) + IS);
            M85 = mS*d*sin(ALPHAT + PHI);
            M86 = mS*d*VT*cos(ALPHAT + PHI);
            M87 = - (mS*(d^2 + (b+c)*d*cos(PHI)) + IS);
            M88 = mS*d^2 + IS;

            M = [   1 0 0 0  0   0   0   0 ;...
                    0 1 0 0  0   0   0   0 ;...
                    0 0 1 0  0   0   0   0 ;...
                    0 0 0 1  0   0   0   0 ;...
                    0 0 0 0 M55 M56 M57 M58 ;...
                    0 0 0 0 M65 M66 M67 M68 ;...
                    0 0 0 0 M75 M76 M77 M78 ;...
                    0 0 0 0 M85 M86 M87 M88 ];
        end
    end
end

%% See Also
%
% <../../index.html Home>
%
