clear,clc

%% Dados do veiculo
a = 1.4;   % distancia do eixo dianteiro ao centro de massa [m]
b = 1.8;   % distancia do eixo dianteiro ao centro de massa [m]
DELTA = 0*pi/180; % Esterçamento

l = 0.7;    % Largura do veiculo para animação [m]

%% Integração do sistema
T = 10; % Tempo de simulação

TSPAN = 0:0.1:T;
x0 = [0 ; 45*pi/180 ; 20]; % Condição inicial dos estados
x0 = [x0 ; 0]; % Condição da orientacao
x0 = [x0 ; 0 ; 0]; % Condição inicial da trajetória



DADOS=[a b DELTA];

% Função ja inclui o cálculo da trajetória
[TOUT,XOUT] = ode45(@(t,x) nonlinandrefun(t,x,DADOS) ,TSPAN,x0); 




dPSI = XOUT(:,1);
ALPHAT = XOUT(:,2);
v = XOUT(:,3);

numF = (v.*sin(ALPHAT) + a*dPSI);
numR = (v.*sin(ALPHAT) - b*dPSI);
den = (v.*cos(ALPHAT));

% for i=1:length(ALPHAT)
%  
% ALPHAF(i) = atan(numF(i)/den(i)) - DELTA ;
% ALPHAR(i) = atan(numR(i)/den(i));
%     
%     
%     if den(i)<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
%         if numF(i)>0
%             ALPHAF(i) = atan(numF(i)/den(i)) - DELTA +pi;
%         else
%             ALPHAF(i) = atan(numF(i)/den(i))-pi;
%         end
%         
%         if numR(i)>0
%             ALPHAR(i) = atan(numR(i)/den(i))+pi;
%         else
%             ALPHAR(i) = atan(numR(i)/den(i))-pi;
%         end
%     end
% end

for i=1:length(ALPHAT)
 
ALPHAF(i) = atan(numF(i)/den(i)) - DELTA ;
ALPHAR(i) = atan(numR(i)/den(i));
    
    
    if den(i)<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
        ALPHAF(i) = -atan(numF(i)/den(i)) - DELTA;
        ALPHAR(i) = -atan(numR(i)/den(i));
    end
end

%% Resultados
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
hold on
plot(XOUT(:,5),XOUT(:,6),'r')
title('Trajetória')
xlabel('X [m]')
ylabel('Y [m]')

figure(4)
hold on
plot(TOUT,XOUT(:,4)*180/pi,'r')
title('Orientacao')
xlabel('tempo [s]')
ylabel('PSI [grau]')


figure(5)
hold on
plot(TOUT,ALPHAF*180/pi,'r')
plot(TOUT,ALPHAR*180/pi,'g')
title('Angulos de deriva')
xlabel('Tempo [s]')
ylabel('Angulo [grau]')
legend('Frente','Tras')



%% FAZER ANIMAÇÃO

PSI = XOUT(:,4);
XT = XOUT(:,5);
YT = XOUT(:,6);
 
    % Vetores posi��o 1, 2, 3 e 4 em rela��o a T na base (T t1 t2 t3)
    rt1t = [a;l]; % dianteira esquerda
    rt2t = [a;-l]; % dianteira direita
    rt3t = [-b;-l]; % traseira direita
    rt4t = [-b;l]; % traseira esquerda

for j=1:length(TOUT)
% Matriz de rota��o da base (T t1 t2 t3) para (o i j k)
RTI=[cos(PSI(j)) -sin(PSI(j));sin(PSI(j)) cos(PSI(j))];
% Vetores posi��o 1, 2, 3 e 4 em rela��o a origem do ref inercial
% na base (T t1 t2 t3)
rt1i(j,1:2) = (RTI*rt1t)';
rt2i(j,1:2) = (RTI*rt2t)';
rt3i(j,1:2) = (RTI*rt3t)';
rt4i(j,1:2) = (RTI*rt4t)';
end

% Vetores posi��o 1, 2, 3 e 4 em rela��o a o na base (o i j k)
rc1t=[XT YT]+rt1i;
rc2t=[XT YT]+rt2i;
rc3t=[XT YT]+rt3i;
rc4t=[XT YT]+rt4i;

% Ajuste do tempo

TEMPO = 0:0.05:TOUT(end);

for i=1:length(TEMPO)

rc1(i,1:2) = interp1(TOUT,rc1t,TEMPO(i));
rc2(i,1:2) = interp1(TOUT,rc2t,TEMPO(i));
rc3(i,1:2) = interp1(TOUT,rc3t,TEMPO(i));
rc4(i,1:2) = interp1(TOUT,rc4t,TEMPO(i));

end

f=figure(6);
set(f,'Units','centimeters')
set(f,'Position',[1 1 15 15])

ax=gca();
hold on
axis equal
% set(ax,'Units','centimeters')
% set(ax,'OuterPosition',[1 1 FEa*escX*FEf FEa*escY*FEf])
set(ax,'XLim',[min(XT)-10 max(XT)+10])
set(ax,'XLimMode','manual')
set(ax,'YLim',[min(YT)-10 max(YT)+10])
set(ax,'YLimMode','manual')

% axis([min(XT)-10 max(XT)+10 min(YT)-10 max(YT)+10]);
% axis manual

ti = title('Trajet\''{o}ria - Modelo n\~{a}o linear');
    set(ti,'Interpreter','Latex','FontSize',15); 
xl = xlabel('Dist\^{a}ncia [m]');
    set(xl,'Interpreter','Latex','FontSize',15);
yl = ylabel('Dist\^{a}ncia [m]');
    set(yl,'Interpreter','Latex','FontSize',15); 
    %legend('Tractor','Semitrailer')

% Primeiro frame
xc = [rc1(1,1) rc2(1,1) rc3(1,1) rc4(1,1)];
yc = [rc1(1,2) rc2(1,2) rc3(1,2) rc4(1,2)];

%hold on
fill(xc,yc,'r')


% frame = getframe(6);
% im = frame2im(frame);
% [A,map] = rgb2ind(im,256,'nodither'); 
% imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','LoopCount',Inf,'DelayTime',0.1);

for j = 1:length(TEMPO)
   
plot(XOUT(:,5),XOUT(:,6),'r')

xc = [rc1(j,1) rc2(j,1) rc3(j,1) rc4(j,1)];
yc = [rc1(j,2) rc2(j,2) rc3(j,2) rc4(j,2)];

%hold on
fill(xc,yc,'r')

% frame = getframe(6);
% im = frame2im(frame);
% [A,map] = rgb2ind(im,256,'nodither'); 
% imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','WriteMode','append','DelayTime',0.05);


%    drawnow
%    F(j) = getframe;
    pause(0.05)
 %   ax=gca();
    cla(ax); %clear axes

end

% �ltimo frame

plot(XOUT(:,5),XOUT(:,6),'r')

xc = [rc1(end,1) rc2(end,1) rc3(end,1) rc4(end,1)];
yc = [rc1(end,2) rc2(end,2) rc3(end,2) rc4(end,2)];

fill(xc,yc,'r')




