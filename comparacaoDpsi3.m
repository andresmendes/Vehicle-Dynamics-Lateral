clear all,clc,close all
%% Info
% IMPORTANTE!: Ver Info de simulacao.m!

%% Descrição
% Este script tem como objetivo comparar o resultado das combinações de modelos de pneu e veículos.
% Neste script o ângulo de esterçamento do pneu dianteiro é nulo e as condições iniciais nulas iguais a dPSI = 2 [rad/s] e vy = 0 [m/s]]

% Vai ser o número de figuras
veiculoModeloVet = [1 2 3];
veiculoModeloTxt = char(' Linear 2 GDL',' Não linear 2 GDL',' Não linear 3 GDL');
%veiculoModeloTitulo4 = 'veiculoNaoLinear3gdlExtendido';

veiculoDados = 1; 

% Vai ser o número de curvas em cada figura
pneuModeloVet = [1 2 3]; % Modelos 1-Linear; 2-Sadri; 3-Pacejka
pneuModeloTxt = char('Linar','Sadri','Pacejka');
pneuModeloCor = char('r','g','b');
pneuModeloMarcador = char('o','s','d');
pneuDadosVet = [1 2 3];

for i = 1:length(veiculoModeloVet)
	for j = 1:length(pneuModeloVet)
		
		veiculoModelo = veiculoModeloVet(i);

		pneuModelo = pneuModeloVet(j);
		pneuDados = pneuDadosVet(j);
		
		%% Dados básicos da integração (integrador é chamado mais abaixo)
		% 
		T = 5; % Tempo total de simulação
		TSPAN = 0:T/40:T; % Vetor de tempo de análise

		r0 = 3; % velocidade angular [rad/s]
		vy0 = 0; % velocidade lateral [m/s]
		v = 20; % velocidade longitudinal [m/s] -> ATENÇÃO: Tem que estar de acordo com os dados dos veículos com 2 gdl
		ALPHAT0 = asin(vy0/v); % conversão de vy0 para ALPHAT
		x0 = [r0 ; ALPHAT0]; % Condição inicial dos estados
		x0 = [x0 ; 0]; % Condição da orientacao
		x0 = [x0 ; 0 ; 0]; % Condição inicial da trajetória
		if veiculoModelo == 3
			% Para que o integrador consiga rodar no modelo com 3 gdl é necessário acrescentar uma
			% condição inicial referente ao estado velocidade "v".
			% Ou seja, a velocidade que era prescrita antes agora é condição inicial
			x0 = [x0 ; v]; % Condição inicial da velocidade
		end

		[pneuFun veiculoFun pneuDadosFrente pneuDadosTras veiculoDadosVet pneuTxt veiculoTxt] = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados);

		[TOUT,XOUT] = ode45(@(t,x) veiculoFun(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDadosVet),TSPAN,x0); 

		figure(i)
		hold on
		H1 = plot(TOUT,XOUT(:,1)*180/pi,pneuModeloCor(j));
		set(H1,'Marker',pneuModeloMarcador(j),'MarkerFaceColor',pneuModeloCor(j),'MarkerSize',7)
		title(strcat('Vel. angular - r0 = 3 [rad/s] - Veículo: ',veiculoModeloTxt(i,:)));
		ylabel('dPSI [grau/s]')
		xlabel('Tempo [s]')
		if j == length(pneuModeloVet)
		legend(pneuModeloTxt(1,:),pneuModeloTxt(2,:),pneuModeloTxt(3,:),'Location','SouthEast')
		end

		figure(i + 10);
		hold on
		axis equal
		H1 = plot(XOUT(:,4),XOUT(:,5),pneuModeloCor(j));
		set(H1,'Marker',pneuModeloMarcador(j),'MarkerFaceColor',pneuModeloCor(j),'MarkerSize',7)
		title(strcat('Trajetória - r0 = 3 [rad/s] - Veículo: ',veiculoModeloTxt(i,:)));
		ylabel('Distância [m]')
		if j == length(pneuModeloVet)
		legend(pneuModeloTxt(1,:),pneuModeloTxt(2,:),pneuModeloTxt(3,:),'Location','SouthEast')
		end

	end
end