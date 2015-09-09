clear,clc,close all
%% Descrição
% Script para simulação de veículo simples para condições iniciais e
% esterçamento constante.

%% Dados do veiculo
% Mesmos dados apresentados por SADRI E WU 2013
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inercia [kg]
b = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
a = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20;     % módulo da velocidade do centro de massa [m/s]

DELTA = 0*pi/180; % esterçamento do eixo dianteiro [grau]

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

%% Integração do sistema
T = 50; % Tempo de simulação
TSPAN = 0:0.1:T;

r0 = 10; % velocidade angular [rad/s]
vy0 = 0; % velocidade lateral [m/s]
ALPHAT0 = vy0/v;
x0 = [r0 ; ALPHAT0]; % Condição inicial dos estados
x0 = [x0 ; 0]; % Condição da orientacao
x0 = [x0 ; 0 ; 0]; % Condição inicial da trajetória

% Função ja inclui o cálculo da trajetória
[TOUT,XOUT] = ode45(@(t,x) linearpacejkafun(t,x,VEICULO,PNEU),TSPAN,x0); 

save('resultados','XOUT','TOUT')

%% Estruturação das saídas
dPSI = XOUT(:,1);
ALPHAT = XOUT(:,2); 

% Ângulos de deriva
ALPHAF = ALPHAT + a*dPSI/v - DELTA; % Dianteiro
ALPHAR = ALPHAT - b*dPSI/v;         % Traseiro

% Modulo do vetor velocidade
VF = sqrt((ALPHAT + a*dPSI).^2 + (v)^2); % Dianteiro
VR = sqrt((ALPHAT - b*dPSI).^2 + (v)^2);         % Traseiro
VT = ones(length(VF),1)*v; % Centro de massa T

% numF = (v.*sin(ALPHAT) + a*dPSI);
% numR = (v.*sin(ALPHAT) - b*dPSI);
% den = (v.*cos(ALPHAT));
%  
% 
% for i=1:length(ALPHAT)
%  
% ALPHAF(i) = atan(numF(i)/den(i)) - DELTA ;
% ALPHAR(i) = atan(numR(i)/den(i));
%     
%     
%     if den(i)<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
%         ALPHAF(i) = -atan(numF(i)/den(i)) - DELTA;
%         ALPHAR(i) = -atan(numR(i)/den(i));
%     end
% end
%% Autovalores da matriz jacobiana
% Para verificar a evolução dos autovalores ao longo do tempo

for i=1:length(XOUT)
dPSI = XOUT(i,1);
ALPHAT = XOUT(i,2);
% Componentes
J11 = ((CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1))/I;
J12 = ((CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1) - (CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1))/I;
J21 = -(m*v - (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1) + (CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1))/(m*v);
J22 = ((CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*(ALPHAT - (b*dPSI)/v)^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy)))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*(ALPHAT - (b*dPSI)/v))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)))*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(ALPHAT - DELTA + (a*dPSI)/v)^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy)))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(ALPHAT - DELTA + (a*dPSI)/v))/(CyF*FzF*muy))^2 + 1))/(m*v);
% Matriz      
J =[J11 J12;...
    J21 J22];

valor(i,1:2) = eig(J)';
end

%% Resultados
figure(1)
hold on
plot(TOUT,XOUT(:,1),'r')
plot(TOUT,XOUT(:,2)*180/pi,'g')
title('Estados X Tempo')
xlabel('Tempo [s]')
ylabel('[rad] ou [rad/s]')
legend('dPSI','ALPHAT')

figure(2)
hold on
plot(TOUT,XOUT(:,3)*180/pi,'r')
title('Orientacao X Tempo')
xlabel('tempo [s]')
ylabel('Orientacao [grau]')

figure(3)
hold on
plot(XOUT(:,4),XOUT(:,5),'r')
title('Trajetoria')
xlabel('Distancia [m]')
ylabel('Distancia [m]')

figure(4)
hold on
plot(TOUT,ALPHAF*180/pi,'r')
plot(TOUT,ALPHAR*180/pi,'g')
title('Angulo de deriva')
xlabel('tempo [s]')
ylabel('angulo [grau]')
legend('F','R')

figure(5)
hold on
plot(TOUT,real(valor(:,1)),'r')
plot(TOUT,real(valor(:,2)),'g')
title('Parte real dos autovalores')
xlabel('Tempo [s]')
xlabel('Autovalor')
legend('1','2')

%% Animação
cd ..   % Voltando uma pasta
cd animacao % Entrando na pasta de animação
animacao(XOUT,TOUT,ALPHAF,ALPHAR,VF,VR,VT,VEICULO); % Executando o script de animação
cd .. % Saindo da pasta de animação
cd linearpacejka % voltando para a pasta original