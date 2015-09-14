clear,clc
%% Descrição
% Este script tem como objetivo fazer o tratamento adequado do ângulo de deriva para uso em condições extremas

%% Dados
% Veículo
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20; % Módulo do vetor velocidade [m/s]
DELTA = 0*pi/180; % esterçamento do eixo dianteiro [rad]

% Variáveis de estado
dPSImax = 4; % Máximo valor de dPSI [rad/s]
ALPHATmax = pi; % Máximo valor de ALPHAT [rad]
res = 100; % Resolução do grid
dPSIvet = -dPSImax:2*dPSImax/res:dPSImax; % Velocidade angular [rad/s]
ALPHATvet = -ALPHATmax:2*ALPHATmax/res:ALPHATmax; % Velocidade lateal (Sadri e Wu 2013) [m/s]

%% Loop principal
% Cálculo dos ângulos de deriva

for i = 1:length(dPSIvet)
	for j = 1:length(ALPHATvet)


		ALPHAFnaolin(i,j) = atan((v*sin(ALPHATvet(j)) + a*dPSIvet(i))/(v*cos(ALPHATvet(j)))) - DELTA; % Dianteiro não linear

		den = cos(ALPHATvet(j));
		
		if den<=0
		ALPHAFnaolin(i,j) = -atan((v*sin(ALPHATvet(j)) + a*dPSIvet(i))/(v*cos(ALPHATvet(j)))) - DELTA; % Dianteiro não linear
    	end
		ALPHAFlin(i,j) = ALPHATvet(j) + a*dPSIvet(i)/v - DELTA; % Dianteiro linear
	end
end

    
    % if (v*cos(ALPHAT))<=0
    %     ALPHAF = -atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
    %    ALPHAR = -atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));
    % end

%% Resultados

figure(1)
s = surface(ALPHATvet*180/pi,dPSIvet,ALPHAFnaolin*180/pi);