clear,clc,close all

%% Descrição
% Este scipt tem como objetivo obter o valor do ângulo para um dado valor de erro de linearização do ângulo e o seu seno.
% Entende-se por condições extremas aquelas que as hipóteses de linearização não são atingidas. Neste caso, erros acima
% de 10% são considerados extremos. Condições aceitáveis são consideradas como aquelas em que o erro não passa de 1%.

% Para erro de 10%
% anguloSolucao = 45 graus

% Para erro de 1%
% anguloSolucao = 14 graus


%% Equacionamento
% Temos que o erro é dado por:
% 	erro = (angulo - sin(angulo)) / angulo 	(1)
% Logo:
% 	angulo*(1 - erro) = sin(angulo) 		(2)

%% Cálculo

erro = 0.01; % Erro desejado [%]

angulo = (0:0.1:60)*pi/180; % Ângulo de deriva [rad]
reta = (1 - erro)*angulo; % Lado esquerdo da equação (2)
sangulo = sin(angulo); % Lado direito da equação (2)

dif = abs(reta - sangulo); % Módulo da diferença das curvas

[valor,indice] = min(dif(2:end)); % O menor valor da diferença exluindo o valor em zero. Pois ja se sabe que os dois partem do mesmo valor

anguloSolucao = angulo(indice)*180/pi % Ângulo que satisfaz aproximadamente a solução

%% Resultados
% Gráficamente 
figure(1)
hold on
plot(angulo*180/pi,reta,'r')
plot(angulo*180/pi,sangulo,'g')
title('Curvas da equação (2) - Ver código')
xlabel('Ângulo [graus]')

% Para erro de 10%
% anguloSolucao = 45 graus

% Para erro de 1%
% anguloSolucao = 14 graus