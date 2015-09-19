function dx = veiculoNaoLinear2gdl(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDados)

% Dados do veículo
m = veiculoDados(1); % massa do veiculo [kg]
I = veiculoDados(2); % momento de inercia [kg]
a = veiculoDados(3); % distancia do eixo dianteiro ao centro de massa [m]
b = veiculoDados(4); % distancia do eixo dianteiro ao centro de massa [m]
v = veiculoDados(5); % módulo da velocidade do centro de massa [m/s]

DELTA = veiculoDados(6);

% Estados

dPSI = x(1);
ALPHAT = x(2);
PSI = x(3);

% Angulos de deriva não linear
ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA; % Dianteiro
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));         % Traseiro

% Curva característica
FyF = pneuFun(ALPHAF,pneuDadosFrente);
FyR = pneuFun(ALPHAR,pneuDadosTras);

% Equações de estado
dx(1,1) = (FyF*cos(DELTA)*a - FyR*b)/I; % ddPSI
dx(2,1) = (FyF*cos(DELTA) + FyR - m*v*cos(ALPHAT)*dPSI)/(m*v*cos(ALPHAT)); % dALPHAT

% Obtenção da orientação
dx(3,1) = dPSI; % dPSI

% Equações adicionais para o posicionamento (Não necessárias para a dinâmica em guinada)
dx(4,1) = v*cos(ALPHAT + PSI); % X
dx(5,1) = v*sin(ALPHAT + PSI); % Y