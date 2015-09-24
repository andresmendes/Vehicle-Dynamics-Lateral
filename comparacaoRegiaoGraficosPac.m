clear all,clc, close all
%% Descrição
% Este script tem como objetivo comparar a região de estabilidade de modelo de pneu pacejka com modelo de veiculo linear 2gdl e não linear 3gdl

%% Opções:
% Ver Opções de simulacao.m

% Vai ser o número de figuras
veiculoModeloVet = [1 3];
veiculoModeloTxt = char(' Linear 2 GDL',' Não linear 2 GDL',' Não linear 3 GDL');
veiculoModeloTxtSave = char('L2GDL','NL2GDL','NL3GDL'); % Texto para o arquivo do workspace
%veiculoModeloTitulo4 = 'veiculoNaoLinear3gdlExtendido';

veiculoDados = 1; 

% Vai ser o número de curvas em cada figura
pneuModeloVet = [3]; % Modelos 1-Linear; 2-Sadri; 3-Pacejka
pneuModeloTxt = char(' Linar',' Sadri',' Pacejka');
pneuModeloTxtSave = char('Linar','Sadri','Pacejka');
pneuModeloCor = char('r','g','b');
pneuModeloMarcador = char('o','s','d');
pneuDadosVet = [3];


        % Variação das condições iniciais
        % Definindo o grid

        rr = 8*3; % refino do grid de r
        vv = 40*3; % refino do grid de vy

        total = num2str(rr); % String usado para a descrição do progresso

        rgrid = linspace(-4,4,rr);
        vygrid = linspace(-19,19,vv);

        % Valor total do grid usado para estimar o estágio da simulação
        total = num2str(length(rgrid)); 

        [X,Y] = meshgrid(vygrid,rgrid); % lp = linearpacejka

%% Geração dos gráficos
q = pneuModeloVet
for p = 1:length(veiculoModeloVet)
  
    if p==2
        p=3;
    end

    % Carregando as variáveis usadas nos gráficos
    load(strcat('resultados/regiaoResultados',veiculoModeloTxtSave(p,:),pneuModeloTxtSave(q,:)))

    % Zerando as bordas da matriz Z
    Z(1,:) = 0;
    Z(:,1) = 0;
    Z(end,:) = 0;
    Z(:,end) = 0;
    
    figure(1);
    hold on
    [C,h] = contour(X,Y,Z*p,0.5*p); % As duas multiplicações por q permitem mudar a cor do contorno (meio gambiarra eu acho)
    set(h,'LineWidth',3,'LineStyle','-')
    title(strcat('Região de Estabilidade - Veículo: ',pneuModeloTxt(q,:)));
    ylabel('dPSI [rad/s]')
    xlabel('Velocidade lateral [m/s]')
    if p == length(veiculoModeloVet)
    legend(veiculoModeloTxt(1,:),veiculoModeloTxt(3,:),'Location','SouthEastOutside')
    end

    % title(strcat('Região de estabilidade - Pneu: ',pneuTxt,';',' Veículo: ',veiculoTxt))
    % xlabel('Velocidade lateral [m/s]')
    % ylabel('Velocidade angular [rad/s]')
end