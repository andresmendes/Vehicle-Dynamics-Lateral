classdef PneuPolinomial < DinamicaVeicular.Pneu
    methods
        % constructor
        function self = PneuPolinomial(varargin)
            if nargin == 0
                self.params = [1000 500];
                % k1 - Coeficiente angular
                % k2 - Coeficiente do modelo polinomial
            else
                self.params = varargin{1};
            end
        end

        function Fy = Characteristic(self,alpha)
            % Coeficientes do modelo polinomial
            k1 = self.params(1);
            k2 = self.params(2);
            % Força lateral polinomial
            Fy = - (k1*alpha-k2*alpha^3);
        end
    end

    properties
        params
    end
end
