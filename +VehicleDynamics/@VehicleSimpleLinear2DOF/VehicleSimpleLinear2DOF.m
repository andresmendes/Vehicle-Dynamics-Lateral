%% Nonlinear 2 DOF vehicle model
% Bicycle model nonlinear with 2 degrees of freedom.
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT PSI XT YT VEL]</td> </tr>
% </table> </html>
%
%% Description
% O centro de massa do ve?culo ? dado pelo ponto $T$ e os eixos dianteiro e traseiro s?o dados pelos pontos $F$ e $R$, respectivamente. A constante $a$ mede a dist?ncia do ponto $F$ ao $T$ e $b$ a dist?ncia do ponto $T$ ao $R$. Os ?ngulos $\alpha_F$ e $\alpha_R$ s?o os ?ngulos de deriva nos eixos dianteiro e traseiro. $\alpha_T$ is the vehicle side slip angle and $\psi$ is the vehicle yaw angle. Por fim, $\delta$ ? o ?ngulo de ester?amento.
%
% <<ilustracoes/modeloSimples.svg>>
%
%% Code
%

classdef VehicleSimpleLinear2DOF < VehicleDynamics.VehicleSimple
	methods
        % Constructor
        function self = VehicleSimpleLinear2DOF(varargin)
            if nargin == 0
                % Entrada padr?o dos dados do ve?culo
                % Vehicle data
                m = 2527; % [kg]
                Iz = 6550; % [kgm2]
                lf = 1.37; % [m]
                lr = 1.86; % [m]

                mF0 = lr*m/(lf+lr);                  % Massa sobre o eixo dianteiro [kg]
                mR0 = lf*m/(lf+lr);                  % Massa sobre o eixo traseiro [kg]
                IT = Iz;                 % Momento de in?rcia [kg*m2]
                DELTA = 0;                  % Ester?amento do eixo dianteiro [rad]
                lT = lf+lr;                 % Dist?ncia entre os eixos[m]
                % (Os dados de tire j? s?o do eixo equivalente)
                nF = 1;                     % N?mero de tires no eixo dianteiro
                nR = 1;                     % N?mero de tires no eixo traseiro
                largT = 2;                  % width [m]
                muy = 0.7;                  % Coeficiente de atrito de opera??o
                entradaVetor = [mF0 mR0 IT DELTA lT nF nR largT muy];
                % Definindo os par?metros da classe
                self.params = self.convert(entradaVetor);
                self.tire = VehicleDynamics.TirePolynomial;
            else
                self.params = self.convert(varargin{1});
                self.tire = varargin{2};
            end
                self.distFT = self.params(11);
                self.distTR = self.params(12);
                self.width = self.params(8);
        end

        %% Model
        % Function with the model
        function dx = Model(self,~,estados)
            % Data
            m = self.params(10);        % massa do veiculo [kg]
            Iz = self.params(3);         % momento de inercia [kg]
            lf = self.params(11);        % distancia do eixo dianteiro ao centro de massa [m]
            lr = self.params(12);        % distancia do eixo dianteiro ao centro de massa [m]
            nF = self.params(6);        % N?mero de tires no eixo dianteiro do caminh?o-trator
            nR = self.params(7);        % N?mero de tires no eixo traseiro do caminh?o-trator
            muy = self.params(9);       % Coeficiente de atrito de opera??o
            deltaf = self.params(4);
            g = 9.81;                   % Acelera??o da gravidade [m/s^2]
            vx = 20; % m/
            FzF = self.params(1)*g;     % Carga vertical no eixo dianteiro [N]
            FzR = self.params(2)*g;     % Carga vertical no eixo traseiro [N]

            % States
            vy = estados(1);
            r = estados(2);
            PSI = estados(3);

            % Slip angles
            alphaf = - deltaf + (vy + lf*r)/vx;    % Front
            alphar = (vy - lr*r)/vx;               % Rear

            % Lateral force
            Fyf = nF*self.tire.Characteristic(alphaf,FzF/nF,muy);
            Fyr = nR*self.tire.Characteristic(alphar,FzR/nR,muy);

            % State equations
            dvy = (Fyf*cos(deltaf) + Fyr - m*vx*r)/m;
            dr = (lf*Fyf*cos(deltaf) - lr*Fyr)/Iz;

            % State derivative
            dx(1,1) = dvy;
            dx(2,1) = dr;

            ALPHAT = asin(vy/vx);
            % Additional states for trajectory
            dx(3,1) = r; % dPSI
            dx(4,1) = vx*cos(ALPHAT + PSI); % X
            dx(5,1) = vx*sin(ALPHAT + PSI); % Y

        end

    end

    methods (Static)
        %% convert
        % A fun??o convert adiciona no vetor de entrada ([mF0 mR0 IT DELTA lT nF nR largT muy]) os par?metros restantes do modelo de ve?culo ([mT a b]).
        function parametros = convert(entrada)
            mF0 = entrada(1);       % Massa no eixo dianteiro [kg]
            mR0 = entrada(2);       % Massa no eixo traseiro [kg]
            lT = entrada(5);        % Dist?ncia entre os eixos [m]
            % Convers?o dos dados para os par?metros usados na equa??o de movimento
            mT = mF0 + mR0;         % massa do ve?culo [kg]
            a = mR0/mT*lT;          % Dist?ncia (F-T) [m]
            b = lT - a;             % Dist?ncia (R-T) [m]
            % Sa?da
            parametros = [entrada mT a b];
        end
    end

    %% Properties
    %

    properties
        params
        tire
        distFT
        distTR
        width
    end

end

%% See Also
%
% <index.html Index> | <VehicleArticulatedNonlinear4DOF.html VehicleArticulatedNonlinear4DOF>
%
