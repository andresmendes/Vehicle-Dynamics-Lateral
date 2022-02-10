classdef VehicleArticulatedNonlinear < VehicleDynamicsLateral.VehicleArticulated
    % VehicleArticulatedNonlinear Nonlinear articulated vehicle model.
    %
    % It inherits properties from VehicleArticulated.

    methods
        % Constructor
        function self = VehicleArticulatedNonlinear()
            self.mF0    = 5200;    % Mass at front axle of the tractor without semi-trailer [kg]
            self.mR0    = 2400;    % Mass at rear axle of the tractor without semi-trailer [kg]
            self.mF     = 6000;    % Mass at front axle of the tractor with semi-trailer [kg]
            self.mR     = 10000;   % Mass at rear axle of the tractor with semi-trailer [kg]
            self.mM     = 17000;   % Mass at the axle of the semi-trailer [kg]
            self.IT     = 46000;   % Moment of inertia of the tractor [kg.m2]
            self.IS     = 450000;  % Moment of inertia of the semi-trailer [kg.m2]
            self.lT     = 3.5;     % Wheelbase of the tractor [m]
            self.lS     = 7.7;     % Distance from articulation (fifth wheel) to semi-trailer axle [m]
            self.c      = -0.3;    % Position of articulation (fifth wheel) to semi-trailer axle [m]
            self.nF     = 2;       % Number of wheels at front axle
            self.nR     = 4;       % Number of wheels at rear axle
            self.nM     = 8;       % Number of wheels at semi-trailer axle
            self.wT     = 2.6;     % Tractor width [m]
            self.wS     = 2.4;     % Semi-trailer width [m]
            self.muy    = 0.3;     % Friction
            self.deltaf = 0;       % Steering angle front [rad]
            self.deltar = 0;       % Steering angle rear [rad]
            self.deltam = 0;       % Steering angle semi-trailer [rad]
            self.Fxf    = 0;       % Longitudinal force at front axle [N]
            self.Fxr    = 0;       % Longitudinal force at rear axle [N]
            self.Fxm    = 0;       % Longitudinal force at semi-trailer axle [N]
        end

        function dx = Model(self,t, states,tspan)
            % dx = VehicleModel.Model(t,states,tspan)
            % dx = VehicleModel.MassMatrix(t,states,tspan)
            %
            % Sintax
            % |dx = _VehicleModel_.Model(t,states,tspan)|
            %
            % |dx = _VehicleModel_.MassMatrix(t,states,tspan)|
            %
            % Arguments
            % The following table describes the input arguments:
            %
            % t - Time
            % states - Model state variables: [XT YT PSI PHI VT ALPHAT dPSI dPHI]
            % tspan - Time span

            % Vehicle parameters
            mT  = self.mT;
            mS  = self.mS;
            % IT = self.IT;
            % IS = self.IS;
            a   = self.a;
            b   = self.b;
            c   = self.c;
            d   = self.d;
            e   = self.e;
            nF  = self.nF;
            nR  = self.nR;
            nM  = self.nM;

            g = 9.81;

            FzF = self.mF * g;
            FzR = self.mR * g;
            FzM = self.mM * g;
            muy = self.muy;

            % States
            X       = states(1,1);
            Y       = states(2,1);
            PSI     = states(3,1);
            PHI     = states(4,1);
            VT      = states(5,1);
            ALPHAT  = states(6,1);
            dPSI    = states(7,1);
            dPHI    = states(8,1);

            if isa(self.deltaf,'function_handle')
                % Closed loop steering 
                deltaf = self.deltaf([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.deltaf)>1
                % Open loop variable steering 
                deltaf = interp1(tspan,self.deltaf,t);
            else
                % Open loop single value steering 
                deltaf = self.deltaf;
            end

            if isa(self.deltar,'function_handle')
                % Closed loop steering 
                deltar = self.deltar([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.deltar)>1
                % Open loop variable steering 
                deltar = interp1(tspan,self.deltar,t);
            else
                % Open loop single value steering 
                deltar = self.deltar;
            end
            
            if isa(self.deltam,'function_handle')
                % Closed loop steering 
                deltam = self.deltam([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.deltam)>1
                % Open loop variable steering 
                deltam = interp1(tspan,self.deltam,t);
            else
                % Open loop single value steering 
                deltam = self.deltam;
            end
            
            % Slip angles
            ALPHAF = - deltaf + atan2(a*dPSI + VT*sin(ALPHAT), VT*cos(ALPHAT));
            ALPHAR = - deltar + atan2(VT*sin(ALPHAT) - b*dPSI, VT*cos(ALPHAT));
%           ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VT * sin(ALPHAT + PHI) - b * dPSI * cos(PHI) - c * dPSI * cos(PHI)),(VT * cos(ALPHAT + PHI) + b * dPSI * sin(PHI) + c * dPSI * sin(PHI)));
            ALPHAM = - deltam + atan2((d + e)*(dPHI - dPSI) + VT*sin(ALPHAT + PHI) - dPSI*cos(PHI)*(b + c), VT*cos(ALPHAT + PHI) + dPSI*sin(PHI)*(b + c));
            % Longitudinal forces
            % Front
            if isa(self.Fxf,'function_handle')
                FxF = self.Fxf([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.Fxf)>1
                FxF = interp1(tspan,self.Fxf,t);
            else
                FxF = self.Fxf;
            end
            % Rear
            if isa(self.Fxr,'function_handle')
                FxR = self.Fxr([X;Y;PSI;PHI;VT;ALPHAT;dPSI;dPHI],t);
            elseif length(self.Fxr)>1
                FxR = interp1(tspan,self.Fxr,t);
            else
                FxR = self.Fxr;
            end
            % Semitrailer
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

            t2 = cos(ALPHAT);
            t3 = cos(PSI);
            t4 = sin(PHI);
            t5 = sin(PSI);
            t6 = cos(deltam);
            t7 = sin(deltam);
            t8 = PSI+deltaf;
            t9 = PSI+deltar;
            t10 = dPHI.^2;
            t11 = dPSI.^2;
            t12 = ALPHAT+PHI;
            t13 = ALPHAT+PSI;
            t21 = -PHI;
            t22 = -PSI;
            t23 = -deltam;
            t14 = cos(t12);
            t15 = cos(t13);
            t16 = sin(t13);
            t17 = cos(t8);
            t18 = cos(t9);
            t19 = sin(t8);
            t20 = sin(t9);
            t24 = FyM.*d.*t6;
            t25 = FyM.*e.*t6;
            t26 = FxM.*d.*t7;
            t27 = FxM.*e.*t7;
            t28 = PHI+t22;
            t29 = PHI+t23;
            t31 = PSI+deltam+t21;
            t30 = cos(t28);
            t32 = sin(t28);
            t33 = cos(t29);
            t34 = sin(t29);
            t35 = cos(t31);
            t36 = sin(t31);
            t37 = VT.*d.*dPSI.*mS.*t14;
            f = [VT.*t15;VT.*t16;dPSI;dPHI;FxF.*t17+FxM.*t35+FxR.*t18-FyF.*t19-FyM.*t36-FyR.*t20-b.*mS.*t3.*t11-c.*mS.*t3.*t11-d.*mS.*t10.*t30-d.*mS.*t11.*t30+VT.*dPSI.*mS.*t16+VT.*dPSI.*mT.*t16+d.*dPHI.*dPSI.*mS.*t30.*2.0;FxF.*t19+FxM.*t36+FxR.*t20+FyF.*t17+FyM.*t35+FyR.*t18-b.*mS.*t5.*t11-c.*mS.*t5.*t11+d.*mS.*t10.*t32+d.*mS.*t11.*t32-VT.*dPSI.*mS.*t15-VT.*dPSI.*mT.*t15-d.*dPHI.*dPSI.*mS.*t32.*2.0;-t24-t25-t26-t27+t37+FxM.*b.*t34-FyM.*b.*t33+FxM.*c.*t34-FyM.*c.*t33+FyF.*a.*cos(deltaf)-FyR.*b.*cos(deltar)+FxF.*a.*sin(deltaf)-FxR.*b.*sin(deltar)+VT.*b.*dPSI.*mS.*t2+VT.*c.*dPSI.*mS.*t2-b.*d.*mS.*t4.*t10-c.*d.*mS.*t4.*t10+b.*d.*dPHI.*dPSI.*mS.*t4.*2.0+c.*d.*dPHI.*dPSI.*mS.*t4.*2.0;t24+t25+t26+t27-t37-b.*d.*mS.*t4.*t11-c.*d.*mS.*t4.*t11];

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

            t2 = cos(ALPHAT);
            t3 = cos(PHI);
            t4 = sin(ALPHAT);
            t5 = b+c;
            t6 = mS+mT;
            t7 = d.^2;
            t8 = ALPHAT+PHI;
            t9 = ALPHAT+PSI;
            t14 = -IS;
            t15 = -PSI;
            t10 = cos(t8);
            t11 = cos(t9);
            t12 = sin(t8);
            t13 = sin(t9);
            t16 = mS.*t7;
            t17 = b.*d.*mS.*t3;
            t18 = c.*d.*mS.*t3;
            t19 = PHI+t15;
            t20 = -t16;
            t21 = cos(t19);
            t22 = sin(t19);
            t23 = -t17;
            t24 = -t18;
            t25 = t14+t20+t23+t24;
            M = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6.*t11,t6.*t13,-mS.*(b.*t4+c.*t4+d.*t12),d.*mS.*t12,0.0,0.0,0.0,0.0,-VT.*t6.*t13,VT.*t6.*t11,-VT.*mS.*(b.*t2+c.*t2+d.*t10),VT.*d.*mS.*t10,0.0,0.0,0.0,0.0,mS.*(d.*t22.*2.0-t5.*sin(PSI).*2.0).*(-1.0./2.0),mS.*(d.*t21.*2.0+t5.*cos(PSI).*2.0).*(-1.0./2.0),IS+IT+t16+t17.*2.0+t18.*2.0+b.^2.*mS+c.^2.*mS+b.*c.*mS.*2.0,t25,0.0,0.0,0.0,0.0,d.*mS.*t22,d.*mS.*t21,t25,IS+t16],[8,8]);
        end
    end
       
end
