%%  Nonlinear 4 DOF articulated vehicle model
%
%% Sintax
% |dx = _VehicleModel_.Model(~,estados)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>estados</tt></td> <td width="70%">Estados do modelo: [dPSI ALPHAT dPHI VEL PHI PSI XT YT]</td> </tr>
% </table> </html>
%
%% Description
% O ângulo $\psi$ define a orientação do caminhão-trator em relação ao referencial inercial. O estado $\phi$ é o ângulo formado entre o caminhão-trator e o semirreboque. O ângulo $\alpha_T$ é o ângulo de deriva do módulo dianteiro e é formado pelo vetor velocidade do centro de massa e a linha longitudinal do caminhão-trator. Por fim, $v$ é o módulo do vetor velocidade do centro de massa do caminhão-trator. Os pontos $T$ e $S$ são coincidentes com os centros de massa do caminhão-trator e semirreboque, respectivamente. Os pontos F e R são coincidentes com os eixos dianteiro e traseiro do caminhão-trator, respectivamente. M é o ponto que representa o eixo do semirreboque e A é o ponto de articulação ente as duas unidades. As grandezas a, b e c da unidade motora são as distâncias entre os pontos F-T, T-R e R-A, respectivamente. Na unidade movida, d e e definem as distâncias entre os pontos A-S e S-M, respectivamente.
%
% <<ilustracoes/modeloArticulado.svg>>
%
% Este modelo é escrito na forma:
%
% $$ M(x) \dot{x} = f(x)$$
%
% Onde $x$ é o vetor de estados, $M(x)$ é a matriz de massa do sistema e $f(x)$ é uma função vetorial não linear. Logo, é necessária uma função que permita a integração do sistema com a matriz de massa escrita explicitamente. Uma opção é utilizar a função _ode45_. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>
%
%% Code
%

classdef VeiculoArticuladoNaoLinear4GDL < DinamicaVeicular.VeiculoArticulado
	methods
        % Constructor
        function self = VeiculoArticuladoNaoLinear4GDL(varargin)
            if nargin == 0
                % Entrada padrão dos dados do veículo
                mF0 = 5237;         % Massa no eixo dianteiro do caminhão-trator desacoplado [kg]
                mR0 = 2440;         % Massa no eixo traseiro do caminhão-trator desacoplado [kg]
                mF = 6000;          % Massa no eixo dianteiro do caminhão-trator (F) [kg]
                mR = 10000;         % Massa no eixo traseiro do caminhão-trator (R) [kg]
                mM = 17000;         % Massa no eixo do semirreboque (M) [kg]
                IT = 46100;         % Momento de inércia do caminhão-trator [kg*m2]
                IS = 452010;        % Momento de inércia do semirreboque [kg*m2]
                DELTA = 0;          % Esterçamento do eixo dianteiro [rad]
                c = -0.310;         % Distância da articulação ao eixo traseiro do caminhão-trator (A-R) [m]
                lT = 3.550;         % Distância entre os eixos do caminhão-trator [m]
                lS = 7.700;         % Distância entre a articulação e o eixo do semirreboque [m]
                nF = 2;             % Número de pneus no eixo dianteiro do caminhão-trator
                nR = 4;             % Número de pneus no eixo traseiro do caminhão-trator
                nM = 8;             % Número de pneus no eixo do semirreboque
                larguraT = 2.6;     % Largura do caminhão-trator [m]
                larguraS = 2.550;   % Largura do semirreboque [m]
                muy = 0.3;          % Coeficiente de atrito de operação
                entradaVetor = [mF0 mR0 mF mR mM IT IS DELTA c lT lS nF nR nM larguraT larguraS muy];
                % Definindo os parâmetros da classe
                self.params = self.conversao(entradaVetor);
                self.pneu = DinamicaVeicular.PneuPacejka1989;
            else
                self.params = self.conversao(varargin{1});
                self.pneu = varargin{2};
            end
                self.distFT = self.params(20);
                self.distTR = self.params(21);
                self.distRA = self.params(9);
                self.distAS = self.params(22);
                self.distSM = self.params(23);
                self.largura = self.params(15);
                self.larguraSemi = self.params(16);
        end

        %% Model
        % Função com as equações de estado do modelo
        function dx = Model(self,~,estados)
            % Dados do veículo
            mT = self.params(18);       % massa do veiculo [kg]
            mS = self.params(19);       % massa do veiculo [kg]
            % IT = self.params(6);       % momento de inercia [kg]
            % IS = self.params(7);       % momento de inercia [kg]
            a = self.params(20);        % distancia do eixo dianteiro ao centro de massa do caminhão-trator [m]
            b = self.params(21);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            c = self.params(9);         % distancia da articulação ao centro de massa do caminhão-trator [m]
            d = self.params(22);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            e = self.params(23);        % distancia da articulação ao centro de massa do caminhão-trator [m]
            DELTA = self.params(8);     % Esterçamento [rad]
            nF = self.params(12);       % Número de pneus no eixo dianteiro do caminhão-trator
            nR = self.params(13);       % Número de pneus no eixo traseiro do caminhão-trator
            nM = self.params(14);       % Número de pneus no eixo do semirreboque
            g = 9.81;                   % Aceleração da gravidade [m/s^2]
            FzF = self.params(3)*g;     % Carga vertical no eixo dianteiro [N]
            FzR = self.params(4)*g;     % Carga vertical no eixo traseiro [N]
            FzM = self.params(5)*g;     % Carga vertical no eixo do semirreboque [N]
            muy = self.params(17);      % Coeficiente de atrito de operação
            % Definição dos estados
            dPSI = estados(1,1);        % Velocidade angular do caminhão-trator [rad/s]
            ALPHAT = estados(2,1);      % Ângulo de deriva do CG do caminhão-trator [rad]
            dPHI = estados(3,1);        % Velocidade angular relativa entre o semirreboque e o caminhão-trator [rad/s]
            VEL = estados(4,1);         % Ângulo relativo entre o semirreboque e o caminhão-trator [rad]
            PHI = estados(5,1);         % Módulo do vetor velocidade do CG do caminhão-trator [m/s]
            PSI = estados(6,1);         % Ângulo de orientação do caminhão-trator [rad]

            % Angulos de deriva não linear
            ALPHAF = atan2((a*dPSI + VEL*sin(ALPHAT)),(VEL*cos(ALPHAT))) - DELTA;
            ALPHAR = atan2((-b*dPSI + VEL*sin(ALPHAT)),(VEL*cos(ALPHAT)));
            ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VEL*sin(ALPHAT + PHI) - b*dPSI*cos(PHI) - ...
                     c*dPSI*cos(PHI)),(VEL*cos(ALPHAT + PHI) + b*dPSI*sin(PHI) + c*dPSI*sin(PHI)));

            % Forças longitudinais
            FxF = 0;
            FxR = 0;
            FxM = 0;
            % Forças laterais nos pneus - Curva característica
            FyF = nF*self.pneu.Characteristic(ALPHAF,FzF/nF,muy);
            FyR = nR*self.pneu.Characteristic(ALPHAR,FzR/nR,muy);
            FyM = nM*self.pneu.Characteristic(ALPHAM,FzM/nM,muy);

            % ddPSI,dALPHAT,ddPHI,dPHI,dVEL
            f1 = FxR + FxF*cos(DELTA) + FxM*cos(PHI) - FyF*sin(DELTA) + FyM*sin(PHI) - b*dPSI^2*mS - c*dPSI^2*mS + VEL*dPSI*mS*sin(ALPHAT) ...
                 + VEL*dPSI*mT*sin(ALPHAT) - d*dPHI^2*mS*cos(PHI) - d*dPSI^2*mS*cos(PHI) + 2*d*dPHI*dPSI*mS*cos(PHI);
            f2 = FyR + FyF*cos(DELTA) + FyM*cos(PHI) + FxF*sin(DELTA) - FxM*sin(PHI) - VEL*dPSI*mS*cos(ALPHAT) - VEL*dPSI*mT*cos(ALPHAT) + ...
                 d*dPHI^2*mS*sin(PHI) + d*dPSI^2*mS*sin(PHI) - 2*d*dPHI*dPSI*mS*sin(PHI);
            f3 = a*(FyF*cos(DELTA) + FxF*sin(DELTA)) - FyR*b - (b + c)*(d*mS*sin(PHI)*dPHI^2 - 2*d*mS*sin(PHI)*dPHI*dPSI + d*mS*sin(PHI)*dPSI^2 - ...
                 VEL*mS*cos(ALPHAT)*dPSI + FyM*cos(PHI) - FxM*sin(PHI));
            f4 = d*(b*dPSI^2*mS*sin(PHI) - FyM + c*dPSI^2*mS*sin(PHI) + VEL*dPSI*mS*cos(ALPHAT + PHI)) - FyM*e;
            f5 = dPHI;

            f = [f1 ; f2 ; f3 ; f4 ; f5];

            % Equações adicionais para o posicionamento (Não necessárias para a dinâmica em guinada)
            dx6 = dPSI;
            dx7 = VEL*cos(ALPHAT + PSI); % X
            dx8 = VEL*sin(ALPHAT + PSI); % Y

            dx = [f;...
                  dx6;...
                  dx7;...
                  dx8];
        end

        %% Matriz de massa
        %

        function M = MatrizMassa(self,~,estados)
            % Dados do veículo
            mT = self.params(18);       % massa do veiculo [kg]
            mS = self.params(19);       % massa do veiculo [kg]
            IT = self.params(6);        % momento de inercia [kg]
            IS = self.params(7);        % momento de inercia [kg]
            % a = self.params(20);        % distancia do eixo dianteiro ao centro de massa do caminhão-trator [m]
            b = self.params(21);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            c = self.params(9);         % distancia da articulação ao centro de massa do caminhão-trator [m]
            d = self.params(22);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
            % e = self.params(23);        % distancia da articulação ao centro de massa do caminhão-trator [m]
            % DELTA = self.params(8);   % Esterçamento [rad]
            % nF = self.params(12);      % Número de pneus no eixo dianteiro do caminhão-trator
            % nR = self.params(13);      % Número de pneus no eixo traseiro do caminhão-trator
            % nM = self.params(14);      % Número de pneus no eixo do semirreboque
            % g = 9.81;                  % Aceleração da gravidade [m/s^2]
            % FzF = self.params(3)*g;     % Carga vertical no eixo dianteiro [N]
            % FzR = self.params(4)*g;     % Carga vertical no eixo traseiro [N]
            % FzM = self.params(5)*g;     % Carga vertical no eixo do semirreboque [N]
            % muy = self.params(17);      % Coeficiente de atrito de operação
            % Definição dos estados
            % dPSI = estados(1,1);              % Velocidade angular do caminhão-trator [rad/s]
            ALPHAT = estados(2,1);      % Ângulo de deriva do CG do caminhão-trator [rad]
            % dPHI = estados(3,1);              % Velocidade angular relativa entre o semirreboque e o caminhão-trator [rad/s]
            VEL = estados(4,1);         % Ângulo relativo entre o semirreboque e o caminhão-trator [rad]
            PHI = estados(5,1);         % Módulo do vetor velocidade do CG do caminhão-trator [m/s]
            % PSI = estados(6,1);               % Ângulo de orientação do caminhão-trator [rad]
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

            M = [M11 M12 M13 M14 0 0 0 0;...
                 M21 M22 M23 M24 0 0 0 0;...
                 M31 M32 M33 M34 0 0 0 0;...
                 M41 M42 M43 M44 0 0 0 0;...
                  0   0   0   0  1 0 0 0;...
                  0   0   0   0  0 1 0 0;...
                  0   0   0   0  0 0 1 0;...
                  0   0   0   0  0 0 0 1];
        end



    end

    methods (Static)
        %% conversao
        % A função conversao adiciona no vetor de entrada ([mF0 mR0 mF mR mM IT IS DELTA c lT lS nF nR nM larguraT larguraS muy]) os parâmetros restantes do modelo de veículo ([mT mS a b d e]).
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

    %% Properties
    %

    properties
        params
        pneu
        distFT
        distTR
        distRA
        distAS
        distSM
        largura     % Largura do caminhão-trator
        larguraSemi % Largura do semirreboque
    end
end


%% See Also
%
% <index.html Início> | <VeiculoSimpesNaoLinear3GDL.html Nonlinear 3 DOF vehicle model>
%
