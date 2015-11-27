% Iniciando o código
clear all                   % Limpando o workspace
close all                   % Fechando as figuras
clc                         % Limpando o command windows

% DinamicaVeicular            % Adicionando o caminho do package no path do matlab
import DinamicaVeicular.*   % Importando o pacote Dinamica Veicular

% Vetor de tempo TSPAN
T = 2;                      % Tempo total de simulação [s]
resol = 50;                 % Resoluçao
TSPAN = 0:T/resol:T;        % Vetor de tempo de análise
% Condições iniciais x0
dPSI0 = 0.5;                % Velocidade ângular [rad/s]
ALPHAT0 = 0.9;              % Ângulo de deriva em T [rad]
dPHI0 = dPSI0;              % Velocidade ângular relativa entre as duas unidades [rad/s]
VEL0 = 20;                  % Velocidade do centro de massa do caminhão-trator [m/s]
PHI0 = 0;                   % Ângulo formado entre as duas unidades [rad]
PSI0 = 0;                   % Ângulo de orientação do caminhão-trator [rad]
X0 = 0;                     % Posição na direção horizontal [m]
Y0 = 0;                     % Posição na direção vertical [m]
x0 = [dPSI0 ALPHAT0 dPHI0 VEL0 PHI0 PSI0 X0 Y0];

% Parâmetros do pneu
PneuDados = [30e+03 0.8 1.5 -2 3.59 1.33 30e+03 0.2];
% Modelo de pneu
% Opções:
% 01 - PneuLinear
% 02 - PneuPolinomial
% 03 - PneuPacejka
PneuModelo = DinamicaVeicular.PneuPacejka(PneuDados);
% Parâmetros do veículo
VeiculoDados = [5237 2440 6000 10000 17000 46100 452010 0 -0.310 3.550 7.700 2 4 8 2.6 1000];
% Modelo = VeiculoModelo(VeiculoDados,PneuModelo)
% Opções:
% 01 - VeiculoArticuladoNaoLinear4GDL
ModeloSistema = DinamicaVeicular.VeiculoArticuladoNaoLinear4GDL(VeiculoDados,PneuModelo);

% Execução
Simula(x0,TSPAN,ModeloSistema);
