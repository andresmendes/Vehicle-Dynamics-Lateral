%% Template
% Exemplo de script para simulação do veículo alterando os dados de integração, pneu e veículo.
%
%% Início

% Iniciando o código
clear all                   % Limpando o workspace
close all                   % Fechando as figuras
clc                         % Limpando o command windows

import DinamicaVeicular.*   % Importando o pacote Dinamica Veicular

%% Parâmetros de integração

% Tempo de simulação
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

%% Parâmetros do pneu

% Dados
Fz0 = 30e+03;               % Carga vertical nominal
muy0 = 0.8;                 % Coeficiente de atrito nominal
Cy = 1.5;                   % Coeficiente experimental
Ey = -2;                    % Coeficiente experimental
c1 =  3.59;                 % Coeficiente experimental
c2 =  1.33;                 % Coeficiente experimental
PneuDados = [Fz0 muy0 Cy Ey c1 c2];
% Modelo de pneu
% Opções:
% 01 - PneuLinear
% 02 - PneuPolinomial
% 03 - PneuPacejka
PneuModelo = DinamicaVeicular.PneuPacejka(PneuDados);

%% Parâmetros do veículo

% Dados
mF0 = 5237;                 % Massa no eixo dianteiro do caminhão-trator desacoplado [kg]
mR0 = 2440;                 % Massa no eixo traseiro do caminhão-trator desacoplado [kg]
mF = 6000;                  % Massa no eixo dianteiro do caminhão-trator (F) [kg]
mR = 10000;                 % Massa no eixo traseiro do caminhão-trator (R) [kg]
mM = 17000;                 % Massa no eixo do semirreboque (M) [kg]
IT = 46100;                 % Momento de inércia do caminhão-trator [kg*m2]
IS = 452010;                % Momento de inércia do semirreboque [kg*m2]
DELTA = 0;                  % Esterçamento do eixo dianteiro [rad]
c = -0.310;                 % Distância da articulação ao eixo traseiro do caminhão-trator (A-R) [m]
lT = 3.550;                 % Distância entre os eixos do caminhão-trator [m]
lS = 7.700;                 % Distância entre a articulação e o eixo do semirreboque [m]
nF = 2;                     % Número de pneus no eixo dianteiro do caminhão-trator
nR = 4;                     % Número de pneus no eixo traseiro do caminhão-trator
nM = 8;                     % Número de pneus no eixo do semirreboque
largT = 2.6;                % Largura do caminhão-trator [m]
largS = 2.550;              % Largura do semirreboque [m]
muy = 0.3;                  % Coeficiente de atrito de operação
VeiculoDados = [mF0 mR0 mF mR mM IT IS DELTA c lT lS nF nR nM largT largS muy];
% Modelo = VeiculoModelo(VeiculoDados,PneuModelo)
% Opções:
% 01 - VeiculoArticuladoNaoLinear4GDL
ModeloSistema = DinamicaVeicular.VeiculoArticuladoNaoLinear4GDL(VeiculoDados,PneuModelo);

%% Quais gráficos exibir
graf = [0 0 0 0 0 0 1];
% 1 - Faz o gráfico / 0 - Não faz o gráfico
% graf(1) -> Estados
% graf(2) -> Deriva
% graf(3) -> Aceleracao
% graf(4) -> Estados3DVEL
% graf(5) -> Estados3DPHI
% graf(6) -> Trajetória
% graf(6) -> Aceleração

%% Salva a figuras
salvar = 0;

%% Execução

result = Simula(x0,TSPAN,ModeloSistema,graf,salvar);

%% Ver também
%
% <index.html Início>
%
