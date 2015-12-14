%% Linear tire
% Relação linear entre a força lateral e o ângulo de deriva
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
% $$ F_y = K \alpha $$
%
% $F_y$ is the lateral force, $K$ is the cornering stiffness and $\alpha$ is the tire slip angle.
%
% *Hipóteses*
%
% * Relação linear.
% * Válido apenas para pequenos ângulos de deriva.
%
%% Code
%

classdef PneuLinear < DinamicaVeicular.Pneu
    methods
        % Constructor
        function self = PneuLinear(varargin)
            if nargin == 0
                self.params = 1000;
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha)
            Fy = -self.params*alpha;
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
% <index.html Início> | <PneuPolinomial.html Pneu polinomial> | <PneuPacejka1989.html Pneu Pacejka 1989>
%
