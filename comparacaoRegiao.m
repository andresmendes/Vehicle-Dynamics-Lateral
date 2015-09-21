tic % Início da contagem do tempo de simulação
clear,clc,close all
%% Info
% IMPORTANTE!: Ver Info de simulacao.m!

%% Descrição
% Este script tem como objetivo obter a região de estabilidade 

%% Opções:
% Ver Opções de simulacao.m

% Vai ser o número de figuras
veiculoModeloVet = [1 2 3];
veiculoModeloTxt = char(' Linear 2 GDL',' Não linear 2 GDL',' Não linear 3 GDL');
veiculoModeloTxtSave = char('L2GDL','NL2GDL','NL3GDL'); % Texto para o arquivo do workspace
%veiculoModeloTitulo4 = 'veiculoNaoLinear3gdlExtendido';

veiculoDados = 1; 

% Vai ser o número de curvas em cada figura
pneuModeloVet = [1 3]; % Modelos 1-Linear; 2-Sadri; 3-Pacejka
pneuModeloTxt = char(' Linar',' Sadri',' Pacejka');
pneuModeloTxtSave = char('Linar','Sadri','Pacejka');
pneuModeloCor = char('r','g','b');
pneuModeloMarcador = char('o','s','d');
pneuDadosVet = [1 3];

for p = 1:length(veiculoModeloVet)
    for q = 1:length(pneuModeloVet)
        
        veiculoModelo = veiculoModeloVet(p);

        pneuModelo = pneuModeloVet(q);
        pneuDados = pneuDadosVet(q);

        %% Variação das condições iniciais
        %% Definindo o grid

        rr = 8*1; % refino do grid de r
        vv = 40*1; % refino do grid de vy

        total = num2str(rr); % String usado para a descrição do progresso

        rgrid = linspace(-4,4,rr);
        vygrid = linspace(-19,19,vv);

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

                [pneuFun veiculoFun pneuDadosFrente pneuDadosTras veiculoDadosVet pneuTxt veiculoTxt] = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados);

                % As condições iniciais tem três zeros devido a PSI X e Y
                [TOUT,XOUT] = ode45(@(t,x) veiculoFun(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDadosVet),TSPAN,x0); 

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
        save(strcat('resultados/regiaoResultados',veiculoModeloTxtSave(p,:),pneuModeloTxtSave(q,:)))

    end
end

%% Geração dos gráficos

vetor = [0.1 0.5 0.9]

for p = 1:length(veiculoModeloVet)
    for q = 1:length(pneuModeloVet)
    % Carregando as variáveis usadas nos gráficos
    load(strcat('resultados/regiaoResultados',veiculoModeloTxtSave(p,:),pneuModeloTxtSave(q,:)))

    figure(p);
    hold on
    contour(X,Y,Z,vetor(q))
    title(strcat('Região de Estabilidade - Pneu: ',pneuModeloTxt(q,:),';',' Veículo: ',veiculoModeloTxt(p,:)));
    ylabel('dPSI [rad/s]')
    xlabel('Velocidade lateral [m/s]')
    if q == length(pneuModeloVet)
    legend(pneuModeloTxt(1,:),pneuModeloTxt(2,:),pneuModeloTxt(3,:),'Location','SouthEast')
    end

    % title(strcat('Região de estabilidade - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt))
    % xlabel('Velocidade lateral [m/s]')
    % ylabel('Velocidade angular [rad/s]')
    end
end


tempoDeSimulacao = toc % Fim do tempo de simulação 