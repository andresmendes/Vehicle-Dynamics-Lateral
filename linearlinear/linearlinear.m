clear, clc
%% Introdução 
% Este script tem como objetivo simular um veículo simples, sem
% articulação. As características da simulação:
%
% * Modelo bicicleta linear
% * Modelo de pneu linear
% * Esterçamento constante

%% Parâmetros do veículo
% Parâmetros utilizados por (SADRI E WU 2013)

Ca = 57300;             % Parâmetro do pneu retirado do artigo do Sadri e Wu 2013 [N/rad]
CF = 2*Ca;              % Coeficiente de rigidez de curva - Linear equivalente [N/rad]
CR = 2*Ca;              % Coeficiente de rigidez de curva - Linear equivalente[N/rad]
m = 2527;               % massa do veiculo [kg]
I = 6550;               % momento de inercia [kg]
a = 1.37;               % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;               % distancia do eixo dianteiro ao centro de massa [m]
v = 20;                 % módulo da velocidade do centro de massa [m/s]
C1 = CF*a^2 + CR*b^2;   % Coeficiente auxiliar 1
C2 = CF*a - CR*b;       % Coeficiente auxiliar 2
C3 = CF + CR;           % Coeficiente auxiliar 3

%% Definindo o sistema linear
% As matrizes A, B, C e D do sistema:

% Matriz dinâmica do sistema
A = [-C1/(v*I)       -C2/I;...
     -C2/(v^2*m)-1   -C3/(v*m)];
% Matriz de entradas
B = [a*CF/I;...
     CF/(v*m)];
% Matriz de saídas
C = eye(2);
% Matriz de transmissão direta
D = [0;0];
% Sistema linear - Espaço de estados
sym = ss(A,B,C,D);

%% Simulação
% Integrando o sistema para entradas constantes de esterçamento.

step = 0.1;                         % Passo da simulação
time = 2;                           %tempo de simulação
t = 0:step:time;                    % tempo de simulação
DELTA = 0*pi/180*ones(length(t),1); % Esterçamento fixo
x0 = [20;0];                        % Condições iniciais

% Simulação - Integração
[Y,T,X] = lsim(sym,DELTA,t,x0);

%% Resultados
% Evolução dos estados no tempo.

figure(1)
hold on
plot(T,Y)
title('Estados X Tempo')
xlabel('Tempo [t]')
ylabel('Velocidade angular [rad/s], Angulo [rad]')
legend('Velocidade angular','Angulo de deriva de T','Location','Northeast')