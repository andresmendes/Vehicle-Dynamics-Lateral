%% Pneu Pacejka Estendido
% Relação não linear entre a força lateral e o ângulo de deriva. Este
% modelo difere do modelo de pneu Pacejka devido ao tratamento do ângulo de
% deriva acima de 90 graus. A partir deste valor a direção de avanço sobre
% a curva característica se inverte. O ângulo utilizado no modelo de pneu
% deve ser igual a zero quando o ângulo de deriva no método convencional
% for igual a 180. Isto deve ser feito, pois a força lateral com 180 graus
% de ângulo de deriva deve ser igual a zero e não máxima como ocorre no
% modelo sem tratamento. Isto é obtido através da linha:
%
% _ALPHA = asin(sin(deriva)) % [rad]_
%
% Em que o ângulo de deriva sem tratamento "deriva" da origem ao ângulo com
% tratamento "ALPHA" que será usado no cálculo da força lateral.
%
%% Equacionamento
%
% A equação que descreve este modelo é dada por:
%
% $$ F_y = - \frac{\mu_y}{\mu_{y0}} \frac{F_z}{F_{z0}} F_{y0}; $$
%
% A equação característica é dada por:
%
% $$ F_{y0} = D \sin(C \arctan(B \alpha_{eq} - E (B \alpha_{eq} - \arctan(B \alpha_{eq})))) $$
%
% Onde $F_{y0}$ é a força lateral nominal e $\alpha_{eq}$ é o ângulo de
% deriva equivalente. $B$, $C$, $D$ e $E$ são coeficientes obtidos
% experimentalmente. As equações auxiliares são mostradas abaixo.
%
% $$ C_{f \alpha} = c_1 c_2 F_{z0} \sin \left( 2 \arctan \left( \frac{F_z}{c_2 F_{z0}} \right) \right) $$
%
% $$ C_{f \alpha 0} = c_1 c_2 F_{z0} \sin \left( 2 \arctan \left( \frac{F_{z0}}{c_2 F_{z0}} \right) \right) $$
%
% $$ \alpha_{eq} = \frac{C_{f \alpha}}{C_{f \alpha 0}} \frac{\mu_{y0}}{\mu_y} \frac{F_{z0}}{F_z} \alpha $$
%
% $$ D = \mu_{y0} F_{z0} $$
%
% $$ B = \frac{C_{f \alpha 0}}{C*D} $$
%
% *Hipóteses*
%
% * Relação não linear.
% * Ângulo vai de -180 a 180 graus.
%
%% Código
% Código da função:

classdef PneuPacejka < DinamicaVeicular.Pneu
    methods
        % constructor
        function self = PneuPacejka(varargin)
            if nargin == 0
                self.params = [30e+03 0.8 1.5 -2 3.59 1.33 30e+03 0.3];
                % Fz0 - Carga vertical nominal
                % muy0 - Coeficiente de atrito nominal
                % Cy - Coeficiente experimental
                % Ey - Coeficiente experimental
                % c1 - Coeficiente experimental
                % c2 - Coeficiente experimental
                % Fz0 - Carga vertical nominal
                % muy0 - Coeficiente de atrito nominal
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha)
                        % Parâmetros nominais
            Fz0 = self.params(1);
            muy0 = self.params(2);
            % Parâmetros do pneu frente
            Cy = self.params(3);
            Ey = self.params(4);
            c1 = self.params(5);
            c2 = self.params(6);
            % Condições de operação
            Fz = self.params(7);
            muy = self.params(8);
            % Ângulo de deriva - Com tratamento
            ALPHA = asin(sin(alpha)); % [rad]
            % Modelo de pneu
            Cfa = c1*c2*Fz0*sin(2*atan(Fz/(c2*Fz0)));       % Cfa em função de Fz
            Cfa0 = c1*c2*Fz0*sin(2*atan(Fz0/(c2*Fz0)));     % Cfa para Fz0
            alphaeq = Cfa/Cfa0*muy0/muy*Fz0/Fz*ALPHA;       % alpha equivalente
            Dy0 = muy0*Fz0;                                 % Coeficiente experimental
            By0 = Cfa0/(Cy*Dy0);                            % Stiffness factor
            % Curva característica nominal
            Fy0 = Dy0*sin(Cy*atan(By0*alphaeq-Ey*(By0*alphaeq-atan(By0*alphaeq))));
            % Força lateral
            Fy = -muy/muy0*Fz/Fz0*Fy0;
        end
    end

    properties
        params
    end
end

%% Ver também
%
% <index.html Início>
%
