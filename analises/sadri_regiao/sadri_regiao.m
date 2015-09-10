clear,clc
%% Descrição
% Este script tem como objetivo gerar novamente os gráficos da região de
% estabilidade

%% Variáveis

load('regiaoresultadosls_2015_09_08') % carregando as variáveis

%% Definindo a figura e configurando

figure(1) % Inicializando a figura
hold on % objetos inseridos são adicionados
contour(Xls,Yls,Zls,0.5,'r') % Contorno da primeira região
title('Regiao de estabilidade - SADRI E WU 2013')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('a=1.37 b=1.86')

