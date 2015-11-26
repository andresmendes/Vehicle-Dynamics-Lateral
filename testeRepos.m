% Iniciando o código

clear all   % Limpando o workspace
close all   % Fechando as figuras
clc         % Limpando o command windows

DinamicaVeicular % Adicionando o caminho do package no path do matlab
import DinamicaVeicular.*


%% Definindo os dados da integração
% Interessante observar que para uma mudança de orientação pura do
% caminhão-trator o mesmo valor de dPSI0 deve ser colocado em dPHI0. Se
% isso não for feito as duas unidades vão girar aparentemente como um corpo
% rígido

T = 10;              % Tempo total de simulação [s]
%T = 30;              % Tempo total de simulação [s]
TSPAN = 0:T/50:T;   % Vetor de tempo de análise

% Condições iniciais
dPSI0 = 0.3;      % Velocidade ângular [rad/s]
ALPHAT0 = 0.5;    % Ângulo de deriva em T [rad]
dPHI0 = dPSI0;      % Velocidade ângular relativa entre as duas unidades [rad/s]
PHI0 = 0;       % Ângulo formado entre as duas unidades [rad]
VEL0 = 20;      % Velocidade do centro de massa do caminhão-trator [m/s]
PSI0 = 0;       % Ângulo de orientação do caminhão-trator [rad]
X0 = 0;         % Posição na direção horizontal [m]
Y0 = 0;         % Posição na direção vertical [m]

% Vetor de condições iniciais
x0 = [dPSI0 ALPHAT0 dPHI0 PHI0 VEL0 PSI0 X0 Y0];

%% Integrando
% Utilizando ode45

%options = odeset('RelTol',1e-4,'AbsTol',[1e-4 1e-4 1e-5]);

v = VeiculoArticuladoNaoLinear4GDL;

[TOUT,XOUT] = ode45(@(t, estados) v.Model(t, estados),TSPAN,x0);
