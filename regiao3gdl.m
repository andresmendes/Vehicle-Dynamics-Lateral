tic % Início da contagem do tempo de simulação
clear,clc,close all
%% Info
% IMPORTANTE!: Ver Info de simulacao.m!

%% Descrição
% Este script tem como objetivo obter a região de estabilidade 

%% Opções:
% Ver Opções de simulacao.m

% Seleção
pneuModelo = 3; % Escolha do modelo de pneu
pneuDados = 3; % Escolha dos dados do pneu

veiculoModelo = 4; % Escolha do modelo de veículo
veiculoDados = 1; % Escolha dos dados do veículo

%% Seletor
% Definindo as variáveis necessárias para a integração
[pneuFun veiculoFun pneuDadosFrente pneuDadosTras veiculoDadosVet pneuTxt veiculoTxt] = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados);

%% Variação das condições iniciais
%% Definindo o grid

dPSIref = 8*5; % refino do grid de r
ALPHATref = 4*7; % refino do grid de vy



total = num2str(dPSIref); % String usado para a descrição do progresso

dPSIgrid = linspace(-4,4,dPSIref);
ALPHATgrid = linspace(-pi,pi,ALPHATref);

% Valor total do grid usado para estimar o estágio da simulação
total = num2str(length(dPSIgrid)); 

[X,Y] = meshgrid(ALPHATgrid,dPSIgrid); % lp = linearpacejka


% Gerando figura
figure(3)            
hold on

n = 1 % Contador para indice da saida dos estados na geração das trajetorias

for i=1:length(dPSIgrid)
    for j=1:length(ALPHATgrid)
    	
    	T = 10; % Tempo total de simulação
		TSPAN = 0:T/30:T; % Vetor de tempo de análise

    	% Condições iniciais
    	dPSI0 = dPSIgrid(i); % velocidade angular [rad/s]
		ALPHAT0 = ALPHATgrid(j); % velocidade lateral [m/s]
		v = 20; % velocidade longitudinal [m/s] -> ATENÇÃO: Tem que estar de acordo com os dados dos veículos com 2 gdl
		x0 = [dPSI0 ; ALPHAT0]; % Condição inicial dos estados
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

        % As condições iniciais tem três zeros devido a PSI X e Y
        [TOUT,XOUT] = ode45(@(t,x) veiculoFun(t,x,pneuFun,pneuDadosFrente,...
                    pneuDadosTras,veiculoDadosVet),TSPAN,x0); 

% A curva de nível vai ser dada pelo valor para o qual o ALPHAT convergiu! Ou seja para qual valor do ponto fixo

        Z(i,j) = XOUT(end,2); % Útimo valor do vetor solução do ALPHAT

        % Plotar algumas trajetórias
        if rem(i,3)==0 & rem(j,3)==0
%            H1 = plot(XOUT(:,2),XOUT(:,1));
            XX(:,n) = XOUT(:,2);
            YY(:,n) = XOUT(:,1);

            n = n + 1; % Soma um pra proxima slavar na coluna do lado
        end


        % % Verificação do maior valor de ALPHAT
        % ALPHATmax = max(abs(XOUT(:,2)));

        % % Mapeamento do atendimento da condição de estabilidade dada por:
        % % * ALPHAT < 90 graus
        % if ALPHATmax < (pi/2)
        %     Z(i,j) = 1;
        % else
        %     Z(i,j) = 0;
        % end

        % % Caso o integrador falhe - Comum com o pneu sadri
        % if length(TOUT) < length(TSPAN)
        %     Z(i,j) = 0;
        % end
        
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
hold on
[C,h] = contour(X,Y,Z,'ShowText','on');
set(h,'LevelList',[-2*pi -pi 0 pi 2*pi]+0.00001)
set(h,'TextList',[-2*pi -pi  0 pi 2*pi]+0.00001)

for q=1:n
    if XX(end,q) < 0.1 && XX(end,q) > -0.1 % Em torno de zero
        plot(XX(:,q),YY(:,q),'r');
    end

    if XX(end,q) < (pi + 0.1) && XX(end,q) > (pi - 0.1) % Em torno de pi/2
        plot(XX(:,q),YY(:,q),'g');
    end

    if XX(end,q) > (-pi - 0.1) && XX(end,q) < (-pi + 0.1) % Em torno de pi/2
        plot(XX(:,q),YY(:,q),'b');
    end


end

plot(0,0,'k+')
plot(pi,0,'k+')
plot(-pi,0,'k+')
title(strcat('Região de estabilidade - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt))
xlabel('ALPHAT [rad]')
ylabel('Velocidade angular [rad/s]')

f2 = figure(2);
hold on
surface(X,Y,Z);
plot(0,0,'k+')
plot(pi,0,'k+')
plot(-pi,0,'k+')
title(strcat('Região de estabilidade - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt))
xlabel('ALPHAT [rad]')
ylabel('Velocidade angular [rad/s]')
zlabel('ALPHAT final [rad]')


tempoDeSimulacao = toc % Fim do tempo de simulação 

% Salvando todos os dados do workspace para comparação de regiões posteriores
save('regiaoResultados')