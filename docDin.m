%% Descrição
% Script que gera toda a documentação do repositório.

%% Documentação do próprio gerador de documentação

publish('docDin','evalCode',false)

%% Página inicial da documentação

publish('index','evalCode',false)
publish('fluxograma') % Gera o fluxograma ilustrativo
close all % Fechando as figuras geradas

%% Página principal do modelo de pneu

publish('pneuDoc')
close all % Fechando as figuras geradas

    % pneuLinear
    publish('pneuLinearFun','evalCode',false)
    publish('pneuLinearDados','evalCode',false)
    % pneuSadri
    publish('pneuSadriFun','evalCode',false)
    publish('pneuSadriDadosTaylor','evalCode',false)
    publish('pneuSadriDadosAjuste','evalCode',false)
    % pneuPacejka
    publish('pneuPacejkaFun','evalCode',false)
    publish('pneuPacejkaDados','evalCode',false)

%% Página principal do modelo de veículo

publish('veiculoDoc')
close all % Fechando as figuras geradas
clear all % Limpando o workspace
    % veiculoLinear2gdl
    publish('veiculoLinear2gdl','evalCode',false)
    %publish('veiculoLinear2gdlDados','evalCode',false)
    % veiculoNaoLinear2gdl
    publish('veiculoNaoLinear2gdl','evalCode',false)
    %publish('veiculoLinear2gdlDados','evalCode',false)
    % veiculoNaoLinear3gdl
    publish('veiculoNaoLinear3gdl','evalCode',false)
    %publish('veiculoLinear2gdlDados','evalCode',false)
    % veiculoNaoLinear3gdlEst
    publish('veiculoNaoLinear3gdlEst','evalCode',false)
    %publish('veiculoLinear2gdlDados','evalCode',false)

%% Simulações

    % Simulação
    publish('simulacao')
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    % Comparação DELTA 14
    publish('comparacaoDelta14')
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    % Comparação DELTA 45
    publish('comparacaoDelta45')
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    % Comparação dPSI 2
    publish('comparacaoDpsi2')
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    % Comparação dPSI 3
    publish('comparacaoDpsi3')
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace

    
%% Seletor

publish('seletor','evalCode',false)

%% Frame

publish('frame','evalCode',false)

%% Animação

publish('animacao','evalCode',false)
% Vetor
publish('vetor','evalCode',false)


clear all
clc    

cd html % Entrando na pasta da documentação
open index.html % Abrindo a página inicial da documentação
cd .. % Voltando para a pasta raíz do repositório