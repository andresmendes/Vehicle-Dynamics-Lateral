function [ dx ] = naolinearandretratamentofun(t,x,VEICULODADOS,PNEUDADOS)
% Dados o veículo
m = VEICULODADOS(1);   % massa do veiculo [kg]
I = VEICULODADOS(2);   % momento de inercia [kg]
a = VEICULODADOS(3);   % distancia do eixo dianteiro ao centro de massa [m]
b = VEICULODADOS(4);   % distancia do eixo dianteiro ao centro de massa [m]
v = VEICULODADOS(5);     % módulo da velocidade do centro de massa [m/s]
DELTA = VEICULODADOS(6); % Esterçamento

g = 9.81;   % Aceleração da gravidade [m/s2]

% Estados
dPSI = x(1);
ALPHAT = x(2);
PSI = x(3);

% Conversao do angulo para que fique so entre -180 e 180
met = sin(ALPHAT); % verifica se o angulo esta no plano esquerdo ou direito do carro FOLHA 15
if met > 0
    ANGULO = ALPHAT - floor(ALPHAT/(pi))*(pi);
else
    ANGULO = ALPHAT - floor(ALPHAT/(pi))*(pi) - pi;
end
ALPHAT = ANGULO;

% Angulos de deriva não linear
ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA; % Dianteiro
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));         % Traseiro

if cos(ALPHAT)<=0
	ALPHAF = -atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
	ALPHAR = -atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));
end

% Força lateral do modelo de Pacejka
Fz0 = PNEUDADOS(1);
muy0 = PNEUDADOS(2);

% Parâmetros do pneu frente
CyF = PNEUDADOS(3);
EyF = PNEUDADOS(4);
c1F = PNEUDADOS(5);
c2F = PNEUDADOS(6);

% Parâmetros do pneu tras
CyR = PNEUDADOS(7);
EyR = PNEUDADOS(8);
c1R = PNEUDADOS(9);
c2R = PNEUDADOS(10);

% Condições de operação
muy = PNEUDADOS(11);
FzF = PNEUDADOS(12); 
FzR = PNEUDADOS(13);

CfaF = c1F*c2F*Fz0*sin(2*atan(FzF/(c2F*Fz0))); % Cfa em função de Fz
CfaR = c1R*c2R*Fz0*sin(2*atan(FzR/(c2R*Fz0))); % Cfa em função de Fz

Cfa0F = c1F*c2F*Fz0*sin(2*atan(Fz0/(c2F*Fz0))); % Cfa para Fz0
Cfa0R = c1R*c2R*Fz0*sin(2*atan(Fz0/(c2R*Fz0))); % Cfa para Fz0

alphaeqF = CfaF/Cfa0F*muy0/muy*Fz0/FzF*ALPHAF; % alpha equivalente
alphaeqR = CfaR/Cfa0R*muy0/muy*Fz0/FzR*ALPHAR; % alpha equivalente

Dy0F = muy0*Fz0;
Dy0R = muy0*Fz0;

By0F = Cfa0F/(CyF*Dy0F); % Stiffness factor
By0R = Cfa0R/(CyR*Dy0R); % Stiffness factor

Fy0F = Dy0F*sin(CyF*atan(By0F*alphaeqF-EyF*(By0F*alphaeqF-atan(By0F*alphaeqF))));
Fy0R = Dy0R*sin(CyR*atan(By0R*alphaeqR-EyR*(By0R*alphaeqR-atan(By0R*alphaeqR))));

FyF = -muy/muy0*FzF/Fz0*Fy0F;
FyR = -muy/muy0*FzR/Fz0*Fy0R;

% Equações de estado
dx(1,1) = (FyF*cos(DELTA)*a - FyR*b)/I; % ddPSI
dx(2,1) = (FyF*cos(DELTA) + FyR - m*v*cos(ALPHAT)*dPSI)/(m*v*cos(ALPHAT)); % dALPHAT

% Obtenção da orientação
dx(3,1) = dPSI; % dPSI

% Equações adicionais para o posicionamento
    % Não necessárias para a dinâmica em guinada
    
dx(4,1) = v*cos(ALPHAT + PSI); % X
dx(5,1) = v*sin(ALPHAT + PSI); % Y

end