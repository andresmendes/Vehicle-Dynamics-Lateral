%% Polynomial tire
% Relação não linear entre a força lateral e o ângulo de deriva através de uma expressão polinomial.
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% </table> </html>
%
%% Description
%
% A equação que descreve este modelo é dada por:
%
% $$ F_y = k_1 \alpha  - k_2\alpha^3 $$
%
% $F_y$ is the lateral force and $\alpha$ is the tire slip angle. $k_1$ e
% $k_2$ are the model coefficients.
%
% *Hipóteses*
%
% * Relação não linear.
% * Válido apenas até o ângulos de deriva que fornece a máxima força
% lateral (Tire saturation).
%
%% Code
%

classdef PneuPolinomial < DinamicaVeicular.Pneu
    methods
        % Constructor
        function self = PneuPolinomial(varargin)
            if nargin == 0
                k1 = 1000;
                k2 = 500;
                self.params = [k1 k2];
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha)
            % Polynomial model coefficients
            k1 = self.params(1);
            k2 = self.params(2);
            % Lateral force
            Fy = - (k1*alpha-k2*alpha.^3);
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
% <index.html Index> | <PneuLinear.html Pneu linear> | <PneuPacejka1989.html Pneu Pacejka 1989>
%
