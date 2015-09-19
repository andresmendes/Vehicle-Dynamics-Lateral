%% Dados do veiculo
% Mesmos dados apresentados por SADRI E WU 2013
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inércia [kg]
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20;     % módulo da velocidade do centro de massa [m/s]

DELTA = 0*pi/180; % esterçamento do eixo dianteiro [grau]

veiculoDados = [m I a b v DELTA]; 