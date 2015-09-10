clear,clc
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
a = 1.37; % [m]
b = 1.86; % [m]
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

%% Definindo o grid

rr = 160; % refino do grid de r
vv = 500; % refino do grid de vy
rr = 9; % refino do grid de r
vv = 12; % refino do grid de vy
rr = 80;
vv = 400;

rgrid = linspace(-4,4,rr);
vygrid = linspace(-20,20,vv);

total = num2str(length(rgrid)); % Valor total do grid usado para estimar o estágio da simulação

[Xlp,Ylp] = meshgrid(vygrid,rgrid); % lp = linearpacejka

%% Loop principal
% Varredura do grid calculando os expoentes

% Dados para o algoritmo
time = 20; % tempo de simulaçao
step = 0.1; % passo da iteraçao

% for i=1:length(rgrid)
%    for j=1:length(vygrid)
%        [T,Res]=lyapunov2linearpacejka(2,VEICULO,PNEU,0,step,time,[rgrid(i) asin(vygrid(j)/v)],1);
%        L1lp(i,j) = Res(end,1);
%        L2lp(i,j) = Res(end,2);
%    end
% end
% 
% % OBS: A condiçao inicial e manipulada pois o modelo foi desenvolvido para
% % ALPHAT como estado, e nao vy
% 
% for i=1:length(rgrid)
%    for j=1:length(vygrid)
%        %n=isnan(L1(i,j));
%        
%        if L1lp(i,j)<0 & L2lp(i,j)<0
%            Zlp(i,j) = 1;
%        else
%            Zlp(i,j) = 0;
%        end
%    end
% end

T = 20; % Tempo de simulação
TSPAN = 0:0.1:T;

for i=1:length(rgrid)
    for j=1:length(vygrid)
        % As condições iniciais tem três zeros devido a PSI X e Y
        [TOUT,XOUT] = ode45(@(t,x) linearpacejkafun(t,x,VEICULO,PNEU),TSPAN,[rgrid(i) asin(vygrid(j)/v) 0 0 0]);

        ALPHATmax = max(abs(XOUT(:,2)));
        if ALPHATmax < (pi/2)
            Zlp(i,j) = 1;
        else
            Zlp(i,j) = 0;
        end

        if rem(i,8)==0
            clc
            estagio = num2str(i);
            strcat(estagio,'/',total)
        end

    end
end

save('regiaoresultadoslp')

%% Resultados

figure(1)
hold on
contour(Xlp,Ylp,Zlp,0.5)
title('Regiao de estabilidade')
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')
legend('Pacejka')
