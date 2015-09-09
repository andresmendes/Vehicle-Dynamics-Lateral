clear,clc,close all
%% Descrição
% Cálculo dos expoentes de Lyapunov para diversas condições iniciais. A
% partir deles obter a região de estabilidade.
% Influencia no tempo:
    % * tempo de simulação total
    % * numero de iterações
    % * Refino do grid

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
Caf = 57300; % [N/rad]
Car = 57300; % [N/rad]
kf = 4.87;
kr = 4.87;

PNEU = [Caf Car kf kr];

%% Definindo o grid

rr = 160; % refino do grid de r
vv = 500; % refino do grid de vy
rr = 16;
vv = 48;
rr = 80;
vv = 240;
rgrid = linspace(-4,4,rr);
vygrid = linspace(-12,12,vv);

[Xls,Yls] = meshgrid(vygrid,rgrid); % ls = linearsadri

%% Loop principal
% Varredura do grid calculando os expoentes

% Dados para o algoritmo
time = 20; % tempo de simulaçao
step = 0.1; % passo da iteraçao

for i=1:length(rgrid)
   for j=1:length(vygrid)
       [T,Res]=lyapunov2linearsadri(2,VEICULO,PNEU,0,step,time,[rgrid(i) vygrid(j)],1);
       L1ls(i,j) = Res(end,1);
       L2ls(i,j) = Res(end,2);
    end
end

for i=1:length(rgrid)
   for j=1:length(vygrid)
       %n=isnan(L1(i,j));
       
       if L1ls(i,j)<0 & L2ls(i,j)<0
           Zls(i,j) = 1;
       else
           Zls(i,j) = 0;
       end
   end
end

save('regiaoresultadosls','Zls','Xls','Yls','L1ls','L2ls','VEICULO','PNEU','time','step')

%% Resultados

figure(1)
hold on
%contour(X',Y',Z,0.5)
surface(Xls,Yls,Zls)
title('Regiao de estabilidade')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Sadri')

figure(2)
hold on
contour(Xls,Yls,Zls,0.5)
title('Regiao de estabilidade')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Sadri')