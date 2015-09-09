clear,clc
%% Descrição
% Este script tem como objetivo gerar novamente os gráficos da região de
% estabilidade

%% Carregando as variaveis

load('regiaoresultadosls_2015_09_08')

%% Resultados

figure(1)
hold on
%contour(X',Y',Z,0.5)
surface(Xls,Yls,Zls)
title('Regiao de estabilidade')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Sadri')

figure(2)
hold on
contour(Xls,Yls,Zls,0.5)
title('Regiao de estabilidade')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Sadri')