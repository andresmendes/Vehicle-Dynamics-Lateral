clear,clc
%% Descrição
% Calculadora de autovalor para o sistema linear com pneu sadri.
% Os estados são entradas.

%% Dados do veículo
% Por SADRI E WU 2013

m = 2527; % [kg]
I = 6550; % [kgm2]
a = 1.37; % [m]
b = 1.86; % [m]
v = 20; % [m/s]
DELTA = 0*pi/180;

% Conversao para usar os simbolos usados pelo autor
Iz = I;
lf = a;
lr = b;
vx =v;
deltaf = DELTA;

%% Dados do pneu
Caf = 57300; % [N/rad]
Car = 57300; % [N/rad]
kf = 4.87;
kr = 4.87;

%% Estados

r = 0;
vy = 0;

%% Autovalores

J11 = -(2*Car*lr*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*lf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J12 = (2*Car*lr*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) - 2*Caf*lf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J21 = -(m*vx - 2*Car*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;
J22 = -(2*Car*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;
      
J =[J11 J12;...
    J21 J22];

valor = eig(J)
