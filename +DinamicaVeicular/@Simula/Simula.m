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
                self.TSPAN = 0:0.1:10;      % Vetor de tempo de análise
                ModeloInsano = DinamicaVeicular.VeiculoArticuladoNaoLinear4GDL;
                grafico = [1 1 1 1 1 1 1];  % 1 - Verdadeiro / 0 - Falso
                salva = 1;                  % 1 - Verdadeiro / 0 - Falso
            else
                self.x0 = varargin{1};
                self.TSPAN = varargin{2};
                ModeloInsano = varargin{3};
                grafico = varargin{4};
                salva = varargin{5};
            end
            options = odeset('Mass',@ModeloInsano.MatrizMassa);
            [self.TOUT,self.XOUT] = ode45(@(t, estados) ModeloInsano.Model(t, estados),self.TSPAN,self.x0,options);
            g = DinamicaVeicular.Graficos(ModeloInsano);
            % Animação
            if grafico(1) == 1
                g.EstadosArticulado(self.XOUT,self.TOUT);
            end
            if grafico(2) == 1
                g.DerivaArticulado(self.XOUT,self.TOUT);
            end
            if grafico(3) == 1
                g.AceleracaoArticulado(ModeloInsano,self.XOUT,self.TOUT);
            end
            if grafico(4) == 1
                g.Estados3DArticulado1(self.XOUT,self.TOUT);
            end
            if grafico(5) == 1
                g.Estados3DArticulado2(self.XOUT,self.TOUT);
            end
            if grafico(6) == 1
                g.TrajetoriaArticulado(self.XOUT,self.TOUT);
            end
            if grafico(7) == 1
                g.AnimacaoArticulado(self.XOUT,self.TOUT);
            end
            if salva == 1
                % Pegando os handle das figuras
                f1 = figure(1);
                f2 = figure(2);
                f3 = figure(3);
                f4 = figure(4);
                f5 = figure(5);
                f999 = figure(999);
                % Salvando
                print(f1,'-dpdf','Estados.pdf')
                print(f2,'-dpdf','Deriva.pdf')
                print(f3,'-dpdf','Aceleracao.pdf')
                print(f4,'-dpdf','Estados3DVEL.pdf')
                print(f5,'-dpdf','Estados3DPHI.pdf')
                print(f999,'-dpdf','Trajetoria.pdf')
            end
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
