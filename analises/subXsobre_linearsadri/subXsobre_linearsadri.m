clear,clc
%% Descrição
% Este script tem como objetivo gerar novamente os gráficos da região de
% estabilidade

%% Definindo a figura e configurando

figure(1) % Inicializando a figura
hold on % objetos inseridos são adicionados

%% Primeira região

load('regiaoresultadosls_2015_09_08') % carregando as variáveis

contour(Xls,Yls,Zls,0.5,'r') % Contorno da primeira região

%% Carregando as variaveis da segunda região

load('regiaoresultadosls_2015_09_09') % carregando as variáveis

contour(Xls,Yls,Zls,0.5,'g') % Contorno da primeira região

%% Acrescentando os detalhes do gráfico

title('Regiao de estabilidade - SADRI E WU 2013')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('a=1.37 b=1.86','a=1.86 b=1.37')

