clear,clc
%% Descrição
% Calculadora de autovalor para o sistema linear com pneu pacejka.
% Os estados são entradas.

%% Dados do veiculo
% Mesmos dados apresentados por SADRI E WU 2013
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inercia [kg]
a = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20;     % módulo da velocidade do centro de massa [m/s]

DELTA = 0*pi/180; % esterçamento do eixo dianteiro [grau]

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

%% Estados

dPSI = 0;
ALPHAT = 0;

%% Autovalores

J11 = ((CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1))/I;
J12 = ((CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1) - (CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1))/I;
J21 = -(m*v - (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1) + (CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1))/(m*v);
J22 = ((CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1))/(m*v);
      
J =[J11 J12;...
    J21 J22];

valor = eig(J)
