classdef VehicleSimpleNonlinear4DOF < VehicleDynamicsLateral.VehicleSimple
    % VehicleSimpleNonlinear Nonlinear simple vehicle model.
    %
    % It inherits properties from VehicleSimple.

    properties
        FXFRONTLEFT    % Longitudinal force at front left tire [N]
        FXFRONTRIGHT   % Longitudinal force at front right tire [N]
        FXREARLEFT     % Longitudinal force at rear left tire [N]
        FXREARRIGHT    % Longitudinal force at rear right tire [N]
        K             % Torcional stiffness [Nm/rad]
        C              % Torcional damping [Nms/rad]
        H              % CG height [m]
        L              % track width [m]
        % Intertia properties
        IXX
        IYY
        IZZ
        IXY
        IXZ
        IYZ
    end

    methods
        % Constructor
        function self = VehicleSimpleNonlinear4DOF()
            self.mF0 = 700;
            self.mR0 = 600;
            self.IT = 10000;
            self.lT = 3.5;
            self.nF = 2;
            self.nR = 2;
            self.wT = 2;
            self.muy = .8;
            self.deltaf = 0;
            self.FXFRONTLEFT = 0;
            self.FXFRONTRIGHT = 0;
            self.FXREARLEFT = 0;
            self.FXREARRIGHT = 0;

        end

        %% Model
        % Função com as equações de estado do modelo
        function dx = Model(self, t, states,tspan)
            % Data
            m = self.mT;
            a = self.a;
            b = self.b;
            nFrontLeft = self.nF;
            nFrontRight = self.nF;
            nRearLeft = self.nR;
            nRearRight = self.nR;
            muy = self.muy;

            h = self.H;                       % CG height [m]
            l = self.L;                       % track [m]
            Ixx = self.IXX;
            Iyy = self.IYY;
            Izz = self.IZZ;
            Ixy = self.IXY;
            Ixz = self.IXZ;
            Iyz = self.IYZ;

            KK = self.K; % Rigidez torcional da massa suspensa
            CC = self.C;

            g = 9.81;                 % Gravity [m/s^2]

            % Estados
            X = states(1);
            Y = states(2);
            PSI = states(3);
            THETA = states(4);
            v = states(5);
            ALPHAT = states(6);
            dPSI = states(7);
            dTHETA = states(8);

            FzRight = (m*g*l/2 + KK*THETA + CC*dTHETA)/l;
            FzLeft = -(-m*g*l/2 + KK*THETA + CC*dTHETA)/l;

            FzFrontRight = FzRight*b/(a+b);
            FzFrontLeft = FzLeft*b/(a+b);
            FzRearRight = FzRight*a/(a+b);
            FzRearLeft = FzLeft*a/(a+b);

            if isa(self.deltaf,'function_handle')
                DELTA = self.deltaf([X;Y;PSI;v;ALPHAT;dPSI],t);
            elseif length(self.deltaf)>1
                DELTA = interp1(tspan,self.deltaf,t);
            else
                DELTA = self.deltaf;
            end

            % Slip angles

            % Slip angles
            ALPHAFrontLeft = atan2((v*sin(ALPHAT) + dPSI*a),(v*cos(ALPHAT) - dPSI*l/2)) - DELTA;
            ALPHAFrontRight = atan2((v*sin(ALPHAT) + dPSI*a),(v*cos(ALPHAT) + dPSI*l/2)) - DELTA;

            ALPHARearLeft = atan2((v*sin(ALPHAT) - dPSI*b),(v*cos(ALPHAT) - dPSI*l/2));
            ALPHARearRight = atan2((v*sin(ALPHAT) - dPSI*b),(v*cos(ALPHAT) + dPSI*l/2));

            % LONGITUDINAL FORCES
            if isa(self.FXFRONTLEFT,'function_handle')
                FxFrontLeft = self.FXFRONTLEFT([X;Y;PSI;THETA;v;ALPHAT;dPSI;dTHETA],t);
            elseif length(self.FXFRONTLEFT)>1
                FxFrontLeft = interp1(tspan,self.FXFRONTLEFT,t);
            else
                FxFrontLeft = self.FXFRONTLEFT;
            end

            if isa(self.FXFRONTRIGHT,'function_handle')
                FxFrontRight = self.FXFRONTRIGHT([X;Y;PSI;THETA;v;ALPHAT;dPSI;dTHETA],t);
            elseif length(self.FXFRONTRIGHT)>1
                FxFrontRight = interp1(tspan,self.FXFRONTRIGHT,t);
            else
                FxFrontRight = self.FXFRONTRIGHT;
            end

            if isa(self.FXREARLEFT,'function_handle')
                FxRearLeft = self.FXREARLEFT([X;Y;PSI;THETA;v;ALPHAT;dPSI;dTHETA],t);
            elseif length(self.FXREARLEFT)>1
                FxRearLeft = interp1(tspan,self.FXREARLEFT,t);
            else
                FxRearLeft = self.FXREARLEFT;
            end

            if isa(self.FXREARRIGHT,'function_handle')
                FxRearRight = self.FXREARRIGHT([X;Y;PSI;THETA;v;ALPHAT;dPSI;dTHETA],t);
            elseif length(self.FXREARRIGHT)>1
                FxRearRight = interp1(tspan,self.FXREARRIGHT,t);
            else
                FxRearRight = self.FXREARRIGHT;
            end

            % CHARACTERISTIC CURVE
            FyFrontLeft = nFrontLeft * self.tire.Characteristic(ALPHAFrontLeft, FzFrontLeft/nFrontLeft, muy);
            FyFrontRight = nFrontRight * self.tire.Characteristic(ALPHAFrontRight, FzFrontRight/nFrontRight, muy);
            FyRearLeft = nRearLeft * self.tire.Characteristic(ALPHARearLeft, FzRearLeft/nRearLeft, muy);
            FyRearRight = nRearRight * self.tire.Characteristic(ALPHARearRight, FzRearRight/nRearRight, muy);

            % STATE EQUATIONS
            dx(1,1) = v * cos(ALPHAT + PSI);
            dx(2,1) = v * sin(ALPHAT + PSI);
            dx(3,1) = dPSI;
            dx(4,1) = dTHETA;

            dx(5,1) = FxRearLeft + FxRearRight - sin(DELTA)*(FyFrontLeft + FyFrontRight) + cos(DELTA)*(FxFrontLeft + FxFrontRight) + dPSI*m*(v*sin(ALPHAT) - 2*dTHETA*h*cos(THETA));

            dx(6,1) = FyRearLeft + FyRearRight + sin(DELTA)*(FxFrontLeft + FxFrontRight) - m*(h*sin(THETA)*(dTHETA^2 + dPSI^2) + dPSI*v*cos(ALPHAT)) + cos(DELTA)*(FyFrontLeft + FyFrontRight);

            dx(7,1) = Iyz*dPSI^2 - CC*dTHETA - KK*THETA - 2*Iyz*dPSI^2*cos(THETA)^2 + (Iyy*dPSI^2*sin(2*THETA))/2 - (Izz*dPSI^2*sin(2*THETA))/2 + g*h*m*sin(THETA) + dPSI*h*m*v*cos(ALPHAT)*cos(THETA);

            dx(8,1) = Ixy*dTHETA^2 - (Ixz*dPSI^2*sin(2*THETA))/2 - Ixy*dPSI^2*sin(THETA)^2 - FyRearLeft*b*cos(THETA) - FyRearRight*b*cos(THETA) - (FxRearLeft*l*cos(THETA))/2 + (FxRearRight*l*cos(THETA))/2 + 2*Iyz*dTHETA*dPSI*cos(THETA) + Ixx*dTHETA*dPSI*sin(THETA) - Iyy*dTHETA*dPSI*sin(THETA) + Izz*dTHETA*dPSI*sin(THETA) + FyFrontLeft*a*cos(DELTA)*cos(THETA) + FyFrontRight*a*cos(DELTA)*cos(THETA) - (FxFrontLeft*l*cos(DELTA)*cos(THETA))/2 + (FxFrontRight*l*cos(DELTA)*cos(THETA))/2 + FxFrontLeft*a*cos(THETA)*sin(DELTA) + FxFrontRight*a*cos(THETA)*sin(DELTA) + (FyFrontLeft*l*cos(THETA)*sin(DELTA))/2 - (FyFrontRight*l*cos(THETA)*sin(DELTA))/2;


        end



        function M = MassMatrix(self,~,states)
            % Data
            m = self.mT;
            h = self.H;                       % CG height [m]
            Ixx = self.IXX;
            Izz = self.IZZ;
            Ixy = self.IXY;
            Ixz = self.IXZ;
            Iyz = self.IYZ;


            % Estados
            THETA = states(4);
            v = states(5);
            ALPHAT = states(6);

            M = [1 0 0 0 0 0 0 0;...
                 0 1 0 0 0 0 0 0;...
                 0 0 1 0 0 0 0 0;...
                 0 0 0 1 0 0 0 0;...
                 0 0 0 0             m*cos(ALPHAT)            -m*v*sin(ALPHAT)                  h*m*sin(THETA)             0;...
                 0 0 0 0             m*sin(ALPHAT)             m*v*cos(ALPHAT)                             0 -h*m*cos(THETA);...
                 0 0 0 0 -h*m*cos(THETA)*sin(ALPHAT) -h*m*v*cos(ALPHAT)*cos(THETA) -Ixz*cos(THETA)-Ixy*sin(THETA)           Ixx;...
                 0 0 0 0                         0                           0   Izz*cos(THETA)-Iyz*sin(THETA)          -Ixz];

        end
    end
end

%% See Also
%
% <../../index.html Home>
%
