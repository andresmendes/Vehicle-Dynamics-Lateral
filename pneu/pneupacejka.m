%clear,clc
%% Força lateral do modelo de Pacejka
% Parâmetros do pneu

% Condições nominais
Fz0 = 25000; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal

% Coeficientes experimentais
Cy = 1.3;
Ey = -1;
c1 = 3.8;
c2 = 1.33;

% Condições de operação
muy = 0.8;
FzF = 25000;

% Ângulo
ALPHA = (0:0.1:25)*pi/180;

Cfa = c1*c2*Fz0*sin(2*atan(FzF/(c2*Fz0))); % Cfa em função de Fz

Cfa0 = c1*c2*Fz0*sin(2*atan(Fz0/(c2*Fz0))); % Cfa para Fz0

alphaeq = Cfa/Cfa0*muy0/muy*Fz0/FzF*ALPHA; % alpha equivalente

Dy0 = muy0*Fz0;

By0 = Cfa0/(Cy*Dy0); % Stiffness factor

Fy0 = Dy0*sin(Cy*atan(By0*alphaeq-Ey*(By0*alphaeq-atan(By0*alphaeq))));

FyF = muy/muy0*FzF/Fz0*Fy0;

%% Resultados

figure(1)
hold on
plot(ALPHA*180/pi,FyF)
title('Curva característica')
xlabel('angulo de deriva [grau]')
ylabel('Forca lateral [N]')