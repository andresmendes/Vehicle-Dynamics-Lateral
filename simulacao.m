clear all,clc,close all 
%% Info
% Encoding deste arquivo '*.m': windows-1252 (via Sublime3)
% Para rodar em Windows(7)/Matlab 2014a: SEM problemas pois windows-1252 é
% default (Verificar).
% Para rodar em Linux(Ubuntu14.04)/Matlab 2013a: COM problemas pois UTF-8 é
% o default (Verificar),(Nem tem na documentação 2013 UTF-8)
%	* Usar na linha de comando: 'slCharacterEncoding('Windows-1252')'
% 	* Editor não funciona: não exibe os caracteres corretamente (Usar
% 	Sublime3 por exemplo)
% 	* Os gráficos ficam direito.

%% Descrição
% Este sript tem como objetivo simular o comportamento dinâmico de um
% veículo simples escolhendo o modelo de pneu e o modelo bicicleta.
% Dependendo da aplicação este script pode ser usado como função em que os
% parâmetros de entrada são justamente as escolhas de modelos tanto de pneu
% quanto de veículo.

%% Opções:
% Este script possibilita a escolha do modelo de pneu e o modelo de veículo
% Modelos de pneu:
% 	1 - pneuLinearFun
% 	2 - pneuSadriFun
% 	3 - pneuPacejkaFun
% Dados do pneu escolhido:
%	1 - pneuLinearDados
%	2 - pneuSadriDados
%	3 - pneuPacejkaDados
% Modelos de veículo:
% 	1 - veiculoLinear2gdl
% 	2 - veiculoNaoLinear2gdl
% 	3 - veiculoNaoLinear3gdl
% 	4 - veiculoNaoLinear3gdlExt
% Dados do modelo de veículo escolhido:
% 	1 - veiculoDadosScript
% 	2 - veiculoDadosScript14
% 	3 - veiculoDadosScript45

% Seleção
pneuModelo = 3; % Escolha do modelo de pneu
pneuDados = 3; % Escolha dos dados do pneu

veiculoModelo = 4; % Escolha do modelo de veículo
veiculoDados = 1; % Escolha dos dados do veículo

% OBS: para os dados se é uma simulação sem variação de parâmetro os dados são vetores. Se houver variação os parâmetros vem em matrizes onde as colunas são os parâmetros e as linhas o valor deles na variação

%% Dados básicos da integração (integrador é chamado mais abaixo)
% 
T = 3; % Tempo total de simulação
TSPAN = 0:T/30:T; % Vetor de tempo de análise

r0 = 4; % velocidade angular [rad/s]
vy0 = 0; % velocidade lateral [m/s]
v = 20; % velocidade longitudinal [m/s] -> ATENÇÃO: Tem que estar de acordo com os dados dos veículos com 2 gdl
ALPHAT0 = asin(vy0/v); % conversão de vy0 para ALPHAT

ALPHAT0 = -3;

x0 = [r0 ; ALPHAT0]; % Condição inicial dos estados
x0 = [x0 ; 0]; % Condição da orientacao
x0 = [x0 ; 0 ; 0]; % Condição inicial da trajetória
if veiculoModelo == 3
	% Para que o integrador consiga rodar no modelo com 3 gdl é necessário acrescentar uma
	% condição inicial referente ao estado velocidade "v".
	% Ou seja, a velocidade que era prescrita antes agora é condição inicial
	x0 = [x0 ; v]; % Condição inicial da velocidade
end

if veiculoModelo == 4
	% Para que o integrador consiga rodar no modelo com 3 gdl é necessário acrescentar uma
	% condição inicial referente ao estado velocidade "v".
	% Ou seja, a velocidade que era prescrita antes agora é condição inicial
	x0 = [x0 ; v]; % Condição inicial da velocidade
end


%% Seletor
% Definindo as variáveis necessárias para a integração
[pneuFun veiculoFun pneuDadosFrente pneuDadosTras veiculoDadosVet pneuTxt veiculoTxt] = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados);

%% Integrando
% Utilizando a ode45

[TOUT,XOUT] = ode45(@(t,x) veiculoFun(t,x,pneuFun,pneuDadosFrente,...
                    pneuDadosTras,veiculoDadosVet),TSPAN,x0); 

%% Reconstrução dos dados importantes

dPSI = XOUT(:,1);
ALPHAT = XOUT(:,2);

a = veiculoDadosVet(3);
b = veiculoDadosVet(4);
DELTA = veiculoDadosVet(6);

if veiculoModelo == 1
	% Ângulos de deriva	
	ALPHAF = ALPHAT + a*dPSI/v - DELTA; % Dianteiro
	ALPHAR = ALPHAT - b*dPSI/v;         % Traseiro
	% Módulo dos vetores velocidade
	VT = ones(length(dPSI),1)*v; % Centro de massa T

	% Ângulo de deriva para animação
	% Isso é feito pq para os modelos 3 e 4 a contagem do angulo é diferente da orientação do vetor para animação
	ALPHAFA = ALPHAF + DELTA; % Tem que somar o delta por que é o ângulo do vetor vel F em relação ao plano longitudinal
	ALPHARA = ALPHAR; 
end

if veiculoModelo == 2
	% Angulos de deriva não linear
	ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)./(v*cos(ALPHAT))) - DELTA; % Dianteiro
	ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)./(v*cos(ALPHAT)));         % Traseiro
	% Vetor velocidade do centro de massa
	VT = ones(length(dPSI),1)*v; % Centro de massa T

	% Ângulo de deriva para animação
	% Isso é feito pq para os modelos 3 e 4 a contagem do angulo é diferente da orientação do vetor para animação
	ALPHAFA = ALPHAF + DELTA; 
	ALPHARA = ALPHAR; 
end

if veiculoModelo == 3
	% Vetor velocidade do centro de massa
	VT = XOUT(:,6);
	v = VT;
	% Angulos de deriva não linear
	ALPHAF = atan((v.*sin(ALPHAT) + a*dPSI)./(v.*cos(ALPHAT))) - DELTA; % Dianteiro
	ALPHAR = atan((v.*sin(ALPHAT) - b*dPSI)./(v.*cos(ALPHAT)));         % Traseiro

	% Ângulo de deriva para animação
	% Isso é feito pq para os modelos 3 e 4 a contagem do angulo é diferente da orientação do vetor para animação
	ALPHAFA = ALPHAF + DELTA; 
	ALPHARA = ALPHAR; 
end

if veiculoModelo == 4
	% Vetor velocidade do centro de massa
	VT = XOUT(:,6);
	v = VT;

	numF = (v.*sin(ALPHAT) + a*dPSI);
	numR = (v.*sin(ALPHAT) - b*dPSI);
	den = (v.*cos(ALPHAT));
	
	for i=1:length(ALPHAT)

		% Angulos de deriva não linear para plot em função do tempo
		ALPHAF(i) = atan(numF(i)/den(i)) - DELTA ;
		ALPHAR(i) = atan(numR(i)/den(i));
		if den(i)<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
	        ALPHAF(i) = -atan(numF(i)/den(i)) - DELTA;
	        ALPHAR(i) = -atan(numR(i)/den(i));
	    end
	
		% Ângulo de deriva para animação
		% Isso é feito pq para os modelos 3 e 4 a contagem do angulo é diferente da orientação do vetor para animação
		ALPHAFA(i) = atan((v(i)*sin(ALPHAT(i)) + a*dPSI(i))/(v(i)*cos(ALPHAT(i))));
		ALPHARA(i) = atan((v(i)*sin(ALPHAT(i)) - b*dPSI(i))/(v(i)*cos(ALPHAT(i))));
	    if den(i)<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
		    ALPHAFA(i) = atan((v(i)*sin(ALPHAT(i)) + a*dPSI(i))/(v(i)*cos(ALPHAT(i)))) - pi;
    		ALPHARA(i) = atan((v(i)*sin(ALPHAT(i)) - b*dPSI(i))/(v(i)*cos(ALPHAT(i)))) - pi;
	    end
	end
end


% if veiculoModelo == 4

% end

% Modulo do vetor velocidade
VF = sqrt((VT.*sin(ALPHAT) + a*dPSI).^2 + (VT.*cos(ALPHAT)).^2); % Dianteiro
VR = sqrt((VT.*sin(ALPHAT) - b*dPSI).^2 + (VT.*cos(ALPHAT)).^2); % Traseiro

%% Resultados

f1 = figure(1);
% Plotar o gráfico com eixo vertical na esquerda e direita
[AX,H1,H2] = plotyy(TOUT,XOUT(:,1)*180/pi,TOUT,XOUT(:,2)*180/pi); 
% Mudando as cores dos eixos (Padrão vem com cores iguais aos eixos)
set(AX(1),'YColor',[0 0 0]);
set(AX(2),'YColor',[0 0 0]);
set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
set(H2,'Color',[0 1 0],'Marker','s','MarkerFaceColor',[0 1 0],'MarkerSize',7)
title(strcat('Estados X Tempo - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt));
ylabel(AX(1),'dPSI [grau/s]')
ylabel(AX(2),'ALPHAT [grau]')
xlabel('Tempo [s]')
legend('dPSI','ALPHAT');

f2 = figure(2);
H1 = plot(TOUT,XOUT(:,3)*180/pi,'r');
set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
title('Orientação X Tempo')
ylabel('PSI [grau]')
xlabel('Tempo [s]')

f3 = figure(3);
axis equal
H1 = plot(XOUT(:,4),XOUT(:,5),'r');
set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
title('Trajetória')
ylabel('Distância [m]')
xlabel('Distância [m]')

f4 = figure(4);
hold on
H1 = plot(TOUT,ALPHAF*180/pi,'r');
H2 = plot(TOUT,ALPHAR*180/pi,'g');
set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
set(H2,'Color',[0 1 0],'Marker','s','MarkerFaceColor',[0 1 0],'MarkerSize',7)
title('Ângulos de deriva X Tempo')
ylabel('Ângulo [grau]')
xlabel('Tempo [s]')
legend('Frente','Tras')

f5 = figure(5);
hold on
H1 = plot(XOUT(:,2),XOUT(:,1));
set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
plot(0,0,'k+')
plot(0,180,'k+')
plot(0,360,'k+')
plot(0,-180,'k+')
plot(0,-360,'k+')
title('Plano de fase')
xlabel('dPSI [grau/s]')
ylabel('ALPHAT [grau]')
legend('Trajetória','Ponto fixo')


% % figure(5)
% % hold on
% % plot(TOUT,real(valor(:,1)),'r')
% % plot(TOUT,real(valor(:,2)),'g')
% % title('Parte real dos autovalores')
% % xlabel('Tempo [s]')
% % xlabel('Autovalor')
% % legend('1','2')

if veiculoModelo == 3
	f6 = figure(6);
	hold on
	H1 = plot(TOUT,VT,'r');
	set(H1,'Color',[1 0 0],'Marker','o','MarkerFaceColor',[1 0 0],'MarkerSize',7)
	title('Velocidade longitudinal X Tempo ')
	ylabel('Velocidade [m/s]')
	xlabel('Tempo [s]')
end


% %% Salvando as figuras
% print(f1,'resultados/simulacao/estados.pdf','-dpdf')
% print(f2,'resultados/simulacao/orientacao.pdf','-dpdf')
% print(f3,'resultados/simulacao/Trajetoria.pdf','-dpdf')
% print(f4,'resultados/simulacao/deriva.pdf','-dpdf')

% %% Animação
% cd animacao % Entrando na pasta de animação
% % Executando o script de animação
% animacao(XOUT,TOUT,ALPHAFA,ALPHARA,VF,VR,VT,veiculoDadosVet);
% cd .. % Saindo da pasta de animação
