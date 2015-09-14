clear,clc
%% Descrição
% Este script tem como objetivo gerar novamente os gráficos da região de
% estabilidade

%% Definindo a figura e configurando

figure(1) % Inicializando a figura
hold on % objetos inseridos são adicionados

%% Primeira região

load('regiaoresultadoslp_2015_09_09') % carregando as variáveis

contour(Xlp,Ylp,Zlp,0.5,'g') % Contorno da primeira região

%% Segunda região

% carregando as variáveis do outro
load('regiaoresultadosls_2015_09_08') 

contour(Xls,Yls,Zls,0.5,'r') % Contorno da primeira região


%% Acrescentando os detalhes do gráfico

title('Regiao de estabilidade - SADRI x PACEJKA')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Sadri','Pacejka','Location','NorthWest')

