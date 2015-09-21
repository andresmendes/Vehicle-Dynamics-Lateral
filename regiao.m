tic % Início da contagem do tempo de simulação
clear,clc,close all
%% Info
% IMPORTANTE!: Ver Info de simulacao.m!

%% Descrição
% Este script tem como objetivo obter a região de estabilidade 

%% Opções:
% Ver Opções de simulacao.m

% Seleção
pneuModelo = 2; % Escolha do modelo de pneu
pneuDados = 2; % Escolha dos dados do pneu

veiculoModelo = 1; % Escolha do modelo de veículo
veiculoDados = 1; % Escolha dos dados do veículo

%% Seletor
% Definindo as variáveis necessárias para a integração
[pneuFun veiculoFun pneuDadosFrente pneuDadosTras veiculoDadosVet pneuTxt veiculoTxt] = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados);

%% Variação das condições iniciais
%% Definindo o grid

rr = 8*1; % refino do grid de r
vv = 40*1; % refino do grid de vy



total = num2str(rr); % String usado para a descrição do progresso

rgrid = linspace(-4,4,rr);
vygrid = linspace(-12,12,vv);

% Valor total do grid usado para estimar o estágio da simulação
total = num2str(length(rgrid)); 

[X,Y] = meshgrid(vygrid,rgrid); % lp = linearpacejka

for i=1:length(rgrid)
    for j=1:length(vygrid)
    	
    	T = 5; % Tempo total de simulação
		TSPAN = 0:T/30:T; % Vetor de tempo de análise

    	% Condições iniciais
    	r0 = rgrid(i); % velocidade angular [rad/s]
		vy0 = vygrid(j); % velocidade lateral [m/s]
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

        % As condições iniciais tem três zeros devido a PSI X e Y
        [TOUT,XOUT] = ode45(@(t,x) veiculoFun(t,x,pneuFun,pneuDadosFrente,...
                    pneuDadosTras,veiculoDadosVet),TSPAN,x0); 

        % Verificação do maior valor de ALPHAT
        ALPHATmax = max(abs(XOUT(:,2)));

        % Mapeamento do atendimento da condição de estabilidade dada por:
        % * ALPHAT < 90 graus
        if ALPHATmax < (pi/2)
            Z(i,j) = 1;
        else
            Z(i,j) = 0;
        end

        % Progresso da simulação
        % if rem(i,100)==0 & j == 1
        %     clc
        %     estagio = num2str(i);
        %     strcat(estagio,'/',total)
        % end
	end
end

% Salvando os dados do workspace para comparação de regiões posteriores
save('regiaoResultados')

f1 = figure(1);
contour(X,Y,Z,0.5)
title(strcat('Região de estabilidade - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt))
xlabel('Velocidade lateral [m/s]')
ylabel('Velocidade angular [rad/s]')

tempoDeSimulacao = toc % Fim do tempo de simulação 

% Salvando todos os dados do workspace para comparação de regiões posteriores
save('regiaoResultados')