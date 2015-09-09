clear,clc
%% Descrição
% Cálculo dos expoentes de Lyapunov

%% Dados do veículo
% Por SADRI E WU 2013

m = 2527; % [kg]
I = 6550; % [kgm2]
a = 1.37; % [m]
b = 1.86; % [m]
v = 20; % [m/s]
DELTA = 0*pi/180;

% Conversao para usar os simbolos usados pelo autor
lf = a;
lr = b;

VEICULO = [m I a b v DELTA]; 

%% Dados do pneu
Caf = 57300; % [N/rad]
Car = 57300; % [N/rad]
kf = 4.87;
kr = 4.87;

PNEU = [Caf Car kf kr];

%% Expoentes de Lyapunov
% Cálculo utilizando o algoritmo de Wolf 1985

% Dados para o algoritmo
time = 10; % tempo de implementação do algoritmo
step = 0.1; % passo da iteração

r0 = -1.333; % Velocidade angular inical [rad/s]
vy0 = -9.333; % Velocidade lateral [m/s]
x0 = [r0 vy0]; % Condição inicial dos estados

[T,Res]=lyapunov2linearsadri(2,VEICULO,PNEU,0,step,time,x0,1);

figure(1)
plot(T,Res)
title('Expoentes de Lyapunov')
xlabel('Tempo [s]')
ylabel('Expoentes')
legend('1','2')

%RESULTADOS - COMPARANDO COM OS RESULTADOS OBTIDOS POR SADRI E WU 2013
    % Resultado algoritmo Andre:
        % t=100.0000  -6.661281  -6.616164
    % Resultado no artigo de SADRI:    
                     %−6.661     −6.616