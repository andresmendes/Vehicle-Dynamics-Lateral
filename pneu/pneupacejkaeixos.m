clear,clc
%% Descrição
% A curva característica de pacejka varia conforme a carga vertical. Logo,
% para uma determinada distribuição de carga entre os eixos dianteiro e
% traseiro as curvas serão diferentes. Este script exibe as curvas
% características por eixo (dianteiro e traseiro) de um veículo simples.
%% Dados
% Dados do veículo e do pneu

% Veículo
m = 2527;   % massa do veiculo [kg]
a = 1.4;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.8;   % distancia do eixo dianteiro ao centro de massa [m]
g = 9.81;   % Aceleração da gravidade [m/s2]

% Angulo de deriva
amax = 25; % Angulo maximo de analise
% Ângulo
ALPHAF = (0:0.1:amax)*pi/180; % Angulo na dianteira
ALPHAR = (0:0.1:amax)*pi/180; % Angulo na traseira

% Condições nominais
Fz0 = 25000; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal

% Parâmetros do pneu frente
CyF = 0.9;
EyF = -1;
c1F = 5;
c2F = 1.33;

% Parâmetros do pneu tras
CyR = 0.9;
EyR = -1;
c1R = 5;
c2R = 1.33;

% Condições de operação
muy = 0.8;
% Em função da distribuição de carga nos eixos
FzF = m*b/(a+b)*g;
FzR = m*a/(a+b)*g;

CfaF = c1F*c2F*Fz0*sin(2*atan(FzF/(c2F*Fz0))); % Cfa em função de Fz
CfaR = c1R*c2R*Fz0*sin(2*atan(FzR/(c2R*Fz0))); % Cfa em função de Fz

Cfa0F = c1F*c2F*Fz0*sin(2*atan(Fz0/(c2F*Fz0))); % Cfa para Fz0
Cfa0R = c1R*c2R*Fz0*sin(2*atan(Fz0/(c2R*Fz0))); % Cfa para Fz0

alphaeqF = CfaF/Cfa0F*muy0/muy*Fz0/FzF*ALPHAF; % alpha equivalente
alphaeqR = CfaR/Cfa0R*muy0/muy*Fz0/FzR*ALPHAR; % alpha equivalente

Dy0F = muy0*Fz0;
Dy0R = muy0*Fz0;

By0F = Cfa0F/(CyF*Dy0F); % Stiffness factor
By0R = Cfa0R/(CyR*Dy0R); % Stiffness factor

Fy0F = Dy0F*sin(CyF*atan(By0F*alphaeqF-EyF*(By0F*alphaeqF-atan(By0F*alphaeqF))));
Fy0R = Dy0R*sin(CyR*atan(By0R*alphaeqR-EyR*(By0R*alphaeqR-atan(By0R*alphaeqR))));

FyF = muy/muy0*FzF/Fz0*Fy0F;
FyR = muy/muy0*FzR/Fz0*Fy0R;

%% Limita de aderencia
FyFmax = FzF*muy;
FyRmax = FzR*muy;

%% Resultados

figure(10)
hold on
plot(ALPHAF*180/pi,FyF,'r')
plot([0 amax],[FyFmax FyFmax],'r--')
plot(ALPHAR*180/pi,FyR,'g')
plot([0 amax],[FyRmax FyRmax],'g--')
title('Curva característica')
xlabel('angulo de deriva [grau]')
ylabel('Forca lateral [N]')
legend('FyF','FyFmax','FyF','FyFmax')