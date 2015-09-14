clear,clc
%% Descrição

%% Dados
% Veículo
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20; % Módulo do vetor velocidade [m/s]
DELTA = 0*pi/180; % esterçamento do eixo dianteiro [rad]

%% Estados

dPSI = 3.68; % Velocidade angular [rad/s]
ALPHAT = -14.4*pi/180; % Ângulo de deriva do centro de massa T [rad]

%% Angulo de deriva

ALPHAFnaolin = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA % Dianteiro não linear
ALPHAFlin = ALPHAT + a*dPSI/v - DELTA % Dianteiro linear

%% Erro

erro = abs(ALPHAFlin-ALPHAFnaolin) % Erro [rad]
