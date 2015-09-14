clear all,clc,close all
%% Descrição
% Este script tem como objetivo calcular o erro relativo no cálculo do ângulo de deriva devido ao procedimento de linearizão

%% Dados
% Dados necessários para o cálculo do ângulo de deriva

% Veículo
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20; % Módulo do vetor velocidade [m/s]
DELTA = 0*pi/180; % esterçamento do eixo dianteiro [rad]

% Variáveis de estado
dPSImax = 4; % Máximo valor de dPSI [rad/s]
ALPHATmax = pi/2; % Máximo valor de ALPHAT [rad]
res = 100; % Resolução do grid
dPSIvet = -dPSImax:2*dPSImax/res:dPSImax; % Velocidade angular [rad/s]
ALPHATvet = -ALPHATmax:2*ALPHATmax/res:ALPHATmax; % Velocidade lateal (Sadri e Wu 2013) [m/s]

%% Loop principal
% Cálculo dos ângulos de deriva

for i = 1:length(dPSIvet)
	for j = 1:length(ALPHATvet)
		ALPHAFnaolin(i,j) = atan((v*sin(ALPHATvet(j)) + a*dPSIvet(i))/(v*cos(ALPHATvet(j)))) - DELTA; % Dianteiro não linear
		ALPHAFlin(i,j) = ALPHATvet(j) + a*dPSIvet(i)/v - DELTA; % Dianteiro linear
	end
end

%% Cálculo do erro

erro = abs(ALPHAFlin-ALPHAFnaolin);% ./ ALPHAFnaolin); % Erro [rad]


%% Resultados

figure(1)
s = surface(ALPHATvet*180/pi,dPSIvet,erro*180/pi);
ax = get(s,'Parent');
title('Módulo do erro do ângulo de deriva do eixo dianteiro (lin - naolin)')
xlabel('ALPHAT [grau]')
ylabel('dPSI [rad/s]')
zlabel('Módulo do erro [grau]')

figure(2)
[C,h] = contour(ALPHATvet*180/pi,dPSIvet,erro*180/pi,5);
set(h,'ShowText','on','LevelList',[0.1 1 5 10 15]);%,'TextStep',get(h,'LevelStep')*2)
title('Módulo do erro do ângulo de deriva do eixo dianteiro (lin - naolin)')
xlabel('ALPHAT [grau]')
ylabel('dPSI [rad/s]')
legend('Módulo do erro [grau]','Location','NorthEastOutside')
