clear,clc
%% Coeficiente de Lyapunov
% Estados
x1 = 1;
x2 = 45*pi/180;
x3 = 40;
% Variacional
x4 = 1;
x5 = 0;
x6 = 0;
x7 = 0;
x8 = 1;
x9 = 0;
x10 = 0;
x11 = 0;
x12 = 1;

X0 = [x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12];

TSIM = 40;% Tempo de simulação
TLYA = 40; % Tempo do algoritmo de Wolf

[TOUT,XOUT] = ode45('nonlinandrefunext',TSIM,X0);

[T,Res]=lyapunov2(3,@nonlinandrefunext,@ode45,0,0.01,TLYA,[x1 x2 x3],1);

figure(1)
hold on
plot(TOUT,XOUT(:,1)*180/pi,'r')
plot(TOUT,XOUT(:,2)*180/pi,'g')
title('Estados X Tempo')
xlabel('Tempo [s]')
ylabel('[graus] ou [graus/s]')
legend('dPSI','ALPHAT')

figure(2)
hold on
plot(TOUT,XOUT(:,3),'r')
title('Velocidade X Tempo')
xlabel('Tempo [s]')
ylabel('[m/s]')

figure(3)
plot(T,Res)
legend('1','2','3')
% Para  step = 0.001 e tempo = 100 => L1 = -6.616 e L2 = 6.616
%-8.104932  -8.104251   0.000000