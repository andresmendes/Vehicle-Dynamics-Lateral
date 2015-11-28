%% Pneu linear
% Relação linear entre a força lateral e o ângulo de deriva
%
%% Equacionamento
%
% A equação que descreve este modelo é dada por:
%
% $$ F_y = C \alpha $$
%
% Onde $F_y$ é a força lateral, $C$ é o coeficiente de rigidez de curva e
% $\alpha$ é o ângulo de deriva.
%
% *Hipóteses*
%
% * Relação linear.
% * Válido apenas para pequenos ângulos de deriva.
%
%% Código
% Código da classe:

classdef PneuLinear < DinamicaVeicular.Pneu
	methods
        % constructor
        function self = PneuLinear(varargin)
            if nargin == 0
                self.params = 1000;
            else
                self.params = varargin{1};
            end
        end

		function Fy = Characteristic(self,alpha)
            Fy = self.params*alpha;
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
