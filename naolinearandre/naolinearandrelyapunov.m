clear,clc
%% Descrição
% Cálculo dos expoentes de lyapunov

%% Dados do veículo
% Por SADRI E WU 2013

m = 2527; % [kg]
I = 6550; % [kgm2]
b = 1.37; % [m]
a = 1.86; % [m]
v = 20; % [m/s]
DELTA = 0*pi/180;

% Conversao para usar os simbolos usados pelo autor
lf = a;
lr = b;

VEICULO = [m I a b v DELTA]; 

%% Dados do pneu
% Os parâmetros dos eixos dianteiros e trasieros visam ser equivalentes ao
% modelo simplificado usado por SADRI E WU 2013. Isso é feito da seguinte
% utilizando as seguintes equivalencias:
% * Mesmo coeficiente de rigidez de curva para angulos de deriva pequenos
% * Mesma força lateral máxima
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dados retirados do script de pneus (pneusadriXpacejka.m) comparando os dois modelos    
    % muy0 = 0.8;
    % Fz0 = 2.4985e+04;
    % muy = muy0;
    % FzF = Fz0;
    % Cy = 1.5;
    % Ey = -2;
    % c1 = 3.5899;
    % c2 = 1.33;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Fz0 = 2.4985e+04; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal
% Coeficientes experimentais do pneu dianteiro
CyF = 1.5;
EyF = -2;
c1F = 3.5899;
c2F = 1.33;
% Coeficientes experimentaisdo pneu traseiro
CyR = 1.5;
EyR = -2;
c1R = 3.5899;
c2R = 1.33;
% Condições de operação
muy = muy0;
FzF = Fz0; 
FzR = Fz0;

PNEU = [Fz0 muy0 CyF EyF c1F c2F CyR EyR c1R c2R muy FzF FzR];

%% Expoentes de Lyapunov
% Cálculo utilizando o algoritmo de Wolf 1985

% Dados para o algoritmo
time = 10; % tempo de implementação do algoritmo
step = 0.1; % passo da iteração

r0 = 3; % Velocidade angular inical [rad/s]
vy = 0; % Velocidade lateral [m/s]
ALPHAT0 = asin(vy/v); % Ângulo de deriva do centro de massa inicial [rad]
x0 = [r0 ALPHAT0]; % Condição inicial dos estados

[T,Res]=lyapunov2naolinearandre(2,VEICULO,PNEU,0,step,time,x0,1);

figure(1)
plot(T,Res)
legend('1','2')
% Para  step = 0.001 e tempo = 100 => L1 = -6.616 e L2 = 6.616
