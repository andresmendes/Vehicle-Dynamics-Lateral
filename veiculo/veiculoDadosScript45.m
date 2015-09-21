%% Descrição
% Este script define os dados do ḿodelo de veículo.
% Este é um script fixo para os testes com 45 graus de esterçamento.

%% Dados do veiculo
% Mesmos dados apresentados por SADRI E WU 2013
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inércia [kg]
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20;     % módulo da velocidade do centro de massa [m/s]

DELTA = 45*pi/180; % esterçamento do eixo dianteiro [grau]

%% Saída
veiculoDadosVetor = [m I a b v DELTA]; 