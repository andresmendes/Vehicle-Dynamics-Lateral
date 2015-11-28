%% Simulação
% Integração numérica das equações de movimento do modelo de sistema escolhido.

classdef Simula
    methods
        % Constructor
        function self = Simula(varargin)
            if nargin == 0
                % Condições iniciais
                dPSI0 = 0.3;                % Velocidade ângular [rad/s]
                ALPHAT0 = 0.5;              % Ângulo de deriva em T [rad]
                dPHI0 = dPSI0;              % Velocidade ângular relativa entre as duas unidades [rad/s]
                VEL0 = 20;                  % Velocidade do centro de massa do caminhão-trator [m/s]
                PHI0 = 0;                   % Ângulo formado entre as duas unidades [rad]
                PSI0 = 0;                   % Ângulo de orientação do caminhão-trator [rad]
                X0 = 0;                     % Posição na direção horizontal [m]
                Y0 = 0;                     % Posição na direção vertical [m]
                self.x0 = [dPSI0 ALPHAT0 dPHI0 VEL0 PHI0 PSI0 X0 Y0];
                self.TSPAN = 0:0.1:5;       % Vetor de tempo de análise
                ModeloInsano = DinamicaVeicular.VeiculoArticuladoNaoLinear4GDL;
                [self.TOUT,self.XOUT] = ode45(@(t, estados) ModeloInsano.Model(t, estados),self.TSPAN,self.x0);
                g = DinamicaVeicular.Graficos(ModeloInsano);
            else
                self.x0 = varargin{1};
                self.TSPAN = varargin{2};
                % Integrando
                [self.TOUT,self.XOUT] = ode45(@(t, estados) varargin{3}.Model(t, estados),self.TSPAN,self.x0);
                % Definindo os parametros da classe Graficos
                g = DinamicaVeicular.Graficos(varargin{3});
            end
            % Animação
            g.AnimacaoArticulado(self.XOUT,self.TOUT);
            % save('workspace')
        end
    end

    properties
        TSPAN
        x0
        TOUT
        XOUT
    end

end

%% Ver também
%
% <index.html Início>
%
