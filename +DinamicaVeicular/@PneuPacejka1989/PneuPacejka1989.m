%% Pacejka 1989 tire
% Relação não linear entre a força lateral e o ângulo de deriva através de uma expressão semi-empírica com coeficientes experimentais.
%
% Baseado em: BAKKER, E.; PACEJKA, H. B.; LIDNER, L. A new tire model with an application in vehicle dynamics studies. [S.l.], 1989.
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha,Fz,muy)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% <tr> <td width="30%"><tt>Fz</tt></td> <td width="70%">Vertical force [N]</td> </tr>
% <tr> <td width="30%"><tt>muy</tt></td> <td width="70%">Friction coefficient [-]</td> </tr>
% </table> </html>
%
%% Description
% A equação que descreve este modelo é dada por:
%
% $$ F_y = - \frac{\mu_y}{\mu_{y,n}} (F_{y,n}(\alpha_{eq}) + S_v)$$
%
% Onde $\alpha_{eq}$ é o ângulo de deriva equivalente:
%
% $$ \alpha_{eq} = \frac{\mu_{y0}}{\mu_y} \frac{F_{z0}}{F_z} (\alpha + S_h)$$
%
% e $F_{y,n}$ é a equação característica nominal dada por:
%
% $$ F_{y,n} = D \sin(C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha)))) $$
%
% Onde $B$, $C$, $D$ e $E$ são coeficientes do modelo que podem ser obtidos através das seguintes expressões:
%
% $$ C = a_0 $$
%
% $$ D = \mu_{y,n} F_z = (a_1 F_z + a_2) F_z $$
%
% $$ B = \frac{B C D}{C D} = a_3 \sin \left\{ 2 \arctan \left( \frac{F_z}{a_4} \right) \right\} (1 - a_5 |\gamma|)$$
%
% $$ E = a_6 F_z + a_7$$
%
% Os deslocamentos da curva são dados por:
%
% $$ S_h = a_8 \gamma + a_9 F_z + a_{10} $$
%
% $$ S_v = a_{11} F_z \gamma + a_{12} F_z + a_{13} $$
%
% O modelo implementado aqui realiza tratamento do ângulo de deriva para valores acima de 90 graus. A partir deste valor a direção de avanço sobre a curva característica se inverte. O ângulo utilizado no modelo de pneu deve ser igual a zero quando o ângulo de deriva no método convencional for igual a 180. Isto deve ser feito porque a força lateral com 180 graus de ângulo de deriva deve ser igual a zero e não máxima como ocorre no modelo sem tratamento. Isto é obtido através da inclusão da linha de código:
%
% _ALPHA = asin(sin(alpha));_
%
% Em que o ângulo de deriva sem tratamento "alpha" da origem ao ângulo com
% tratamento "ALPHA" que será usado no cálculo da força lateral.
%
% *Hipóteses*
%
% * Relação não linear.
% * Ângulo de deriva vai de -180 a 180 graus.
%
%% Code
%

classdef PneuPacejka1989 < DinamicaVeicular.Pneu
    methods
        % constructor
        function self = PneuPacejka1989(varargin)
            if nargin == 0
                a0 = 1.002806;      % Shape factor [-]
                a1 = 2.014156;      % Load dependency of lateral friction (*1000) [1/kN]
                a2 = 710.5013;      % Lateral friction level (*1000) [-]
                a3 = 5226.341;      % Maximum cornering stiffness [N/deg]
                a4 = 78.87699;      % Load at maximum cornering stiffness [kN]
                a5 = 0.01078379;    % Camber sensitivity of cornering stiffness
                a6 = -0.004759443;  % Load dependency of curvature factor
                a7 = 0.6704447;     % Curvature factor level
                a8 = 0;             % Camber sensitivity of horizontal shift
                a9 = 0;             % Load dependency of horizontal shift
                a10 = 0;            % Horizontal shift level
                a11 = 0;            % Combined load and camber sensitivity of vertical shift
                a12 = 0;            % Load dependency of vertical shift
                a13 = 0;            % Vertical shift level
                self.params = [a0 a1 a2 a3 a4 a5 a6 a7 a8 a9 a10 a11 a12 a13];
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha,Fz,muy)
            % Input
            % alpha - Ângulo de deriva [rad]
            % Fz - Load [N]
            % muy - Lateral friction coefficient (*1000) [-]

            % Tratamento do ângulo de deriva
            ALPHA = asin(sin(alpha));           % [rad]
            ALPHA = 180/pi*ALPHA;               % Conversão [rad] - [deg]
            % Parâmetros nominais
            a0 = self.params(1);
            a1 = self.params(2);
            a2 = self.params(3);
            a3 = self.params(4);
            a4 = self.params(5);
            a5 = self.params(6);
            a6 = self.params(7);
            a7 = self.params(8);
            a8 = self.params(9);
            a9 = self.params(10);
            a10 = self.params(11);
            a11 = self.params(12);
            a12 = self.params(13);
            a13 = self.params(14);

            Fz = Fz/1000;                       % Conversão [N] - [kN]

            camber = 0;                         % Camber angle

            C = a0;                             % Shape factor
            muy0 = a1*Fz + a2;                  % Lateral friction coefficient nominal [-]
            muy = muy*1000;                     % Lateral friction coefficient operacional
            D = muy0*Fz;                        % muy = lateral friction coefficient
            BCD = a3*sin(2*atan(Fz/a4))*(1-a5*abs(camber)); % Cornering stiffness
            E = a6*Fz + a7;                     % Curvature factor
            B = BCD/(C*D);                      % stiffness factor
            Sh = a8*camber + a9*Fz + a10;       % Horizontal shift
            Sv = a11*Fz*camber + a12*Fz + a13;  % Vertical shift
            ALPHAeq = muy0/muy*(ALPHA + Sh);    % Ângulo de deriva equivalente
            % Reference characteristics
            fy = D*sin(C*atan(B*ALPHAeq - E*(B*ALPHAeq - atan(B*ALPHAeq))));
            % Lateral force
            Fy = -muy/muy0*(fy + Sv);
        end
    end

    %% Properties
    %

    properties
        params
    end
end

%% See Also
%
% <index.html Início> | <PneuLinear.html Pneu linear> | <PneuPolinomial.html Pneu polinomial>
%
