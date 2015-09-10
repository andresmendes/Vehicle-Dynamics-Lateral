clear,clc
%% Descrição
% Integração do modelo apresentado por SADRI E WU 2013

%% Dados do veículo
% Por SADRI E WU 2013

m = 2527; % [kg]
I = 6550; % [kgm2]
b = 1.37; % [m]
a = 1.86; % [m]
v = 20; % [m/s]
DELTA = 0*pi/180;

% Conversao para usar os simbolos usados pelo autor
Iz = I;
lf = a;
lr = b;

VEICULO = [m I a b v DELTA]; 

%% Dados do pneu
Caf = 57300; % [N/rad]
Car = 57300; % [N/rad]
kf = 4.87;
kr = 4.87;

PNEU = [Caf Car kf kr];

%% Integração do sistema
TEMPO = 2 ;
tspan = [0 TEMPO];

% Condição inicial do SADRI
r0 = 1.3; % velocidade angular [rad/s]
vy0 = 9.7; % velocidade lateral [m/s]

x0 = [r0;vy0;0;0;0]; % condições iniciais

[TOUT,XOUT] = ode45(@(t,x) linearsadrifun(t,x,VEICULO,PNEU),tspan,x0);


deltaf = DELTA;
r = XOUT(:,1);
vy = XOUT(:,2);
vx = v;

% Alterando o segundo estado. Já que o script de animcação precisa como
% entrada ALPHAT ao invés de vy: ALPHAT = vy/v
estados = XOUT;
estados(:,2) = estados(:,2)/v;

alphaf = (vy+lf*r)/vx-deltaf;
alphar = (vy-lr*r)/vx;

% Módulo dos vetores velocidade
VF = sqrt((vy+lf*r).^2 + (vx)^2);
VR = sqrt((vy-lr*r).^2 + (vx)^2);
VT = ones(length(VF),1)*vx; % Centro de massa T

%% Autovalores da matriz jacobiana
% Para verificar a evolução dos autovalores ao longo do tempo

for i=1:length(XOUT)
r = XOUT(i,1);
vy = XOUT(i,2);
% Componentes
J11 = -(2*Car*lr*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*lf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J12 = (2*Car*lr*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) - 2*Caf*lf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J21 = -(m*vx - 2*Car*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;
J22 = -(2*Car*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;
% Matriz
J = [J11 J12;...
     J21 J22];

valor(i,1:2) = eig(J)';
end

%% Resultados
figure(1)
hold on
plot(TOUT,XOUT(:,2),'r') % velocidade de guinada
plot(TOUT,XOUT(:,1)/v*180/pi,'g') % velocidade lateral // ALPHAT
title('Estados X Tempo')
xlabel('tempo [s]')
ylabel('[rad] ou [rad/s]')
legend('dPSI = r','vy')

figure(2)
plot(TOUT,XOUT(:,3)*180/pi) 
title('Orientacao X Tempo')
xlabel('tempo [s]')
ylabel('Orientacao [grau]')

figure(3)
hold on
plot(XOUT(:,4),XOUT(:,5)) % trajetória dos estados (plano de fase)
title('Trajetoria')
xlabel('Distancia [m]')
ylabel('Distancia [m]')

figure(4)
hold on
plot(TOUT,alphaf*180/pi,'r') % angulo de deriva na dianteira
plot(TOUT,alphar*180/pi,'g') % angulo de deriva na trasiera
title('Angulo de deriva')
xlabel('Tempo [s]')
ylabel('Angulo [grau]')

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
animacao(estados,TOUT,alphaf,alphaf,VF,VR,VT,VEICULO); % Executando o script de animação
cd .. % Saindo da pasta de animação
cd linearsadri % voltando para a pasta original

