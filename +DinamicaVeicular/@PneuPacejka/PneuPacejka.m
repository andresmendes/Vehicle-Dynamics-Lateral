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
