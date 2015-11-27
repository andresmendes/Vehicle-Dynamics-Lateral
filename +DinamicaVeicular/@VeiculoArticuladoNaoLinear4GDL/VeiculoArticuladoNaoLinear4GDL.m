classdef VeiculoArticuladoNaoLinear4GDL < DinamicaVeicular.Veiculo
	methods
        % constructor
        function self = VeiculoArticuladoNaoLinear4GDL(varargin)
            if nargin == 0
                % Entrada padrão dos dados do veículo
                entradaVetor = [5237 2440 6000 10000 17000 46100 452010 0 -0.310 3.550 7.700 2 4 8 2.6 2.550];
                % 01 - mF0 - Massa no eixo dianteiro do caminhão-trator desacoplado [kg]
                % 02 - mR0 - Massa no eixo traseiro do caminhão-trator desacoplado [kg]
                % 03 - mF - Massa no eixo dianteiro do caminhão-trator (F) [kg]
                % 04 - mR - Massa no eixo traseiro do caminhão-trator (R) [kg]
                % 05 - mM - Massa no eixo do semirreboque (M) [kg]
                % 06 - IT - Momento de inércia do caminhão-trator [kg*m2]
                % 07 - IS - Momento de inércia do semirreboque [kg*m2]
                % 08 - DELTA - Esterçamento do eixo dianteiro [rad]
                % 09 - c - Distância da articulação ao eixo traseiro do caminhão-trator (A-R) [m]
                % 10 - lT - Distância entre os eixos do caminhão-trator [m]
                % 11 - lS - Distância entre a articulação e o eixo do semirreboque [m]
                % 12 - nF - Número de pneus no eixo dianteiro do caminhão-trator
                % 13 - nR - Número de pneus no eixo traseiro do caminhão-trator
                % 14 - nM - Número de pneus no eixo do semirreboque
                % 15 - largT - Largura do caminhão-trator [m]
                % 16 - largS - Largura do semirreboque [m]

                % Definindo os parâmetros da classe
                self.params = self.conversao(entradaVetor);
                self.pneu = DinamicaVeicular.PneuPacejka;
            else
                self.params = self.conversao(varargin{1});
                % if  isa(varargin{2}, DinamicaVeicular.Pneu)
                self.pneu = varargin{2};
                % else
                %     % AQUI VAI O ERRO
                %     erro = 1 + 1;
                % end
            end
        end

		function dx = Model(self,~,estados)
            % Dados do veículo
            mT = self.params(17);       % massa do veiculo [kg]
            mS = self.params(18);       % massa do veiculo [kg]
            IT = self.params(6);       % momento de inercia [kg]
            IS = self.params(7);       % momento de inercia [kg]
            a = self.params(19);        % distancia do eixo dianteiro ao centro de massa do caminhão-trator [m]
            b = self.params(20);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            c = self.params(9);        % distancia da articulação ao centro de massa do caminhão-trator [m]
            d = self.params(21);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            e = self.params(22);        % distancia da articulação ao centro de massa do caminhão-trator [m]
            DELTA = self.params(8);   % Esterçamento [rad]
            nF = self.params(12);      % Número de pneus no eixo dianteiro do caminhão-trator
            nR = self.params(13);      % Número de pneus no eixo traseiro do caminhão-trator
            nM = self.params(14);      % Número de pneus no eixo do semirreboque
            g = 9.81;                  % Aceleração da gravidade [m/s^2]
            % Definição dos estados
            dPSI = estados(1,1);              % Velocidade angular do caminhão-trator [rad/s]
            ALPHAT = estados(2,1);            % Ângulo de deriva do CG do caminhão-trator [rad]
            dPHI = estados(3,1);              % Velocidade angular relativa entre o semirreboque e o caminhão-trator [rad/s]
            VEL = estados(4,1);               % Ângulo relativo entre o semirreboque e o caminhão-trator [rad]
            PHI = estados(5,1);               % Módulo do vetor velocidade do CG do caminhão-trator [m/s]
            PSI = estados(6,1);               % Ângulo de orientação do caminhão-trator [rad]

            % Angulos de deriva não linear
            ALPHAF = atan2((a*dPSI + VEL*sin(ALPHAT)),(VEL*cos(ALPHAT))) - DELTA;
            ALPHAR = atan2((-b*dPSI + VEL*sin(ALPHAT)),(VEL*cos(ALPHAT)));
            ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VEL*sin(ALPHAT + PHI) - b*dPSI*cos(PHI) - c*dPSI*cos(PHI)),(VEL*cos(ALPHAT + PHI) + b*dPSI*sin(PHI) + c*dPSI*sin(PHI)));

            % Forças longitudinais
            FxF = 0;
            FxR = 0;
            FxM = 0;
            % Forças laterais nos pneus - Curva característica
            self.pneu.params(7) = self.params(3)*g; % Mudando a carga vertical no eixo dianteiro
            FyF = nF*self.pneu.Characteristic(ALPHAF);
            self.pneu.params(7) = self.params(4)*g; % Mudando a carga vertical no eixo traseiro
            FyR = nR*self.pneu.Characteristic(ALPHAR);
            self.pneu.params(7) = self.params(5)*g; % Mudando a carga vertical no eixo do semirreboque
            FyM = nM*self.pneu.Characteristic(ALPHAM);

            % Matriz de massa
            M11 = -d*mS*sin(PHI);
            M12 = -VEL*sin(ALPHAT)*(mS + mT);
            M13 =  d*mS*sin(PHI);
            M14 = cos(ALPHAT)*(mS + mT);
            M21 = -mS*(b + c + d*cos(PHI));
            M22 = VEL*cos(ALPHAT)*(mS + mT);
            M23 = d*mS*cos(PHI);
            M24 = sin(ALPHAT)*(mS + mT);
            M31 = IT + mS*(b + c)*(b + c + d*cos(PHI));
            M32 = -VEL*mS*cos(ALPHAT)*(b + c);
            M33 = -d*mS*cos(PHI)*(b + c);
            M34 = -mS*sin(ALPHAT)*(b + c);
            M41 = IS + d*mS*(d + cos(PHI)*(b + c));
            M42 = -VEL*d*mS*cos(ALPHAT + PHI);
            M43 = - mS*d^2 - IS;
            M44 = -d*mS*sin(ALPHAT + PHI);

            M = [M11 M12 M13 M14 0;...
                 M21 M22 M23 M24 0;...
                 M31 M32 M33 M34 0;...
                 M41 M42 M43 M44 0;...
                  0   0   0   0  1];

            % ddPSI,dALPHAT,ddPHI,dPHI,dVEL
            f1 = FxR + FxF*cos(DELTA) + FxM*cos(PHI) - FyF*sin(DELTA) + FyM*sin(PHI) - b*dPSI^2*mS - c*dPSI^2*mS + VEL*dPSI*mS*sin(ALPHAT) + VEL*dPSI*mT*sin(ALPHAT) - d*dPHI^2*mS*cos(PHI) - d*dPSI^2*mS*cos(PHI) + 2*d*dPHI*dPSI*mS*cos(PHI);
            f2 = FyR + FyF*cos(DELTA) + FyM*cos(PHI) + FxF*sin(DELTA) - FxM*sin(PHI) - VEL*dPSI*mS*cos(ALPHAT) - VEL*dPSI*mT*cos(ALPHAT) + d*dPHI^2*mS*sin(PHI) + d*dPSI^2*mS*sin(PHI) - 2*d*dPHI*dPSI*mS*sin(PHI);
            f3 = a*(FyF*cos(DELTA) + FxF*sin(DELTA)) - FyR*b - (b + c)*(d*mS*sin(PHI)*dPHI^2 - 2*d*mS*sin(PHI)*dPHI*dPSI + d*mS*sin(PHI)*dPSI^2 - VEL*mS*cos(ALPHAT)*dPSI + FyM*cos(PHI) - FxM*sin(PHI));
            f4 = d*(b*dPSI^2*mS*sin(PHI) - FyM + c*dPSI^2*mS*sin(PHI) + VEL*dPSI*mS*cos(ALPHAT + PHI)) - FyM*e;
            f5 = dPHI;

            f = [f1 ; f2 ; f3 ; f4 ; f5];

            % Equações adicionais para o posicionamento (Não necessárias para a dinâmica em guinada)
            dx6 = dPSI;
            dx7 = VEL*cos(ALPHAT + PSI); % X
            dx8 = VEL*sin(ALPHAT + PSI); % Y

            dx = [M\f;...
                  dx6;...
                  dx7;...
                  dx8];
        end
    end

    methods (Static)
        function parametros = conversao(entrada)
            mF0 = entrada(1);       % Massa no eixo dianteiro do caminhão-trator desacoplado [kg]
            mR0 = entrada(2);       % Massa no eixo traseiro do caminhão-trator desacoplado [kg]
            mF = entrada(3);        % Massa no eixo dianteiro do caminhão-trator (F) [kg]
            mR = entrada(4);        % Massa no eixo traseiro do caminhão-trator (R) [kg]
            mM = entrada(5);        % Massa no eixo do semirreboque (M) [kg]
            lT = entrada(10);       % Distância entre os eixos do caminhão-trator [m]
            lS = entrada(11);       % Distância entre a articulação e o eixo do semirreboque [m]
            % Conversão dos dados para os parâmetros usados na equação de movimento
            g = 9.81;               % Aceleração da gravidade [m/s^2]
            mT = mF0 + mR0;         % massa do caminhão-trator [kg]
            a = mR0/mT*lT;          % Distância do eixo dianteiro ao CG do caminhão-trator (F-T) [m]
            b = lT - a;             % Distância do eixo traseiro ao CG do caminhão-trator (R-T) [m]
            A = mF*g + mR*g - mT*g; % Força vertical na articulação [N]
            mS = (A + mM*g)/g;      % massa do semirreboque [kg]
            d = (lS*mM)/mS;         % Distância da articulação ao CG do semirreboque (A-S) [m]
            e = lS - d;             % Distância do eixo traseiro ao CG do semirreboque (M-S) [m]
            % Saída
            parametros = [entrada mT mS a b d e];
        end
	end

	properties
		params
        pneu
    end
end

% mF0 - Massa no eixo dianteiro do caminhão-trator desacoplado [kg]
% mR0 - Massa no eixo traseiro do caminhão-trator desacoplado [kg]
% mF - Massa no eixo dianteiro do caminhão-trator (F) [kg]
% mR - Massa no eixo traseiro do caminhão-trator (R) [kg]
% mM - Massa no eixo do semirreboque (M) [kg]
% IT - Momento de inércia do caminhão-trator [kg*m2]
% IS - Momento de inércia do semirreboque [kg*m2]
% DELTA - Esterçamento do eixo dianteiro [rad]
% c - Distância da articulação ao eixo traseiro do caminhão-trator (A-R) [m]
% lT - Distância entre os eixos do caminhão-trator [m]
% lS - Distância entre a articulação e o eixo do semirreboque [m]
% nF - Número de pneus no eixo dianteiro do caminhão-trator
% nR - Número de pneus no eixo traseiro do caminhão-trator
% nM - Número de pneus no eixo do semirreboque
% largT - Largura do caminhão-trator [m]
% largS - Largura do semirreboque [m]

% mT - massa do caminhão-trator [kg]
% mS - massa do semirreboque [kg]
% a - Distância do eixo dianteiro ao CG do caminhão-trator (F-T) [m]
% b - Distância do eixo traseiro ao CG do caminhão-trator (R-T) [m]
% d - Distância da articulação ao CG do semirreboque (A-S) [m]
% e - Distância do eixo traseiro ao CG do semirreboque (M-S) [m]
