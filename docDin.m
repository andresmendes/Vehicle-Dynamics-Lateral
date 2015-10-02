%% Gerador de documentação
% Script que gera toda a documentação do repositório.

%% Documentação do próprio gerador de documentação

close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('docDin','outputDir','../documentacao/','evalCode',false)

%% Página inicial da documentação

close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('index','outputDir','../documentacao/','evalCode',false)
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('fluxograma','outputDir','../documentacao/') % Gera o fluxograma ilustrativo

%% Estudos

% Simples
clear all
close all
publish('estudoSimples','outputDir','../documentacao/')

% Comparação DELTA 14
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDelta14','outputDir','../documentacao/')

% Comparação DELTA 45
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDelta45','outputDir','../documentacao/')

% Comparação dPSI 2
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDpsi2','outputDir','../documentacao/')

% Comparação dPSI 3 com
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDpsi3com','outputDir','../documentacao/')

% Comparação dPSI 3 sem
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDpsi3sem','outputDir','../documentacao/')

% Comparação dPSI 3 est
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDpsi3est','outputDir','../documentacao/')

% Comparação dPSI 3 veiculo
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('estudoComparacaoDpsi3veiculo','outputDir','../documentacao/')

%% Página principal do modelo de veículo

% Doc
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('veiculoDoc','outputDir','../documentacao/')
    % veiculoLinear2gdl    
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoLinear2gdl','outputDir','../documentacao/','evalCode',false)
    % veiculoNaoLinear2gdl
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoNaoLinear2gdl','outputDir','../documentacao/','evalCode',false)
    % veiculoNaoLinear3gdl
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoNaoLinear3gdl','outputDir','../documentacao/','evalCode',false)

    % Dados veículo
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoDadosDelta0','outputDir','../documentacao/','evalCode',false)

    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoDadosDelta14','outputDir','../documentacao/','evalCode',false)

    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('veiculoDadosDelta45','outputDir','../documentacao/','evalCode',false)
    
%% PNEU

%Doc
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('pneuDoc','outputDir','../documentacao/')

    % pneuLinear
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuLinearFun','outputDir','../documentacao/','evalCode',false)

    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuLinearDados','outputDir','../documentacao/','evalCode',false)

    % pneuSadri
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuSadriFun','outputDir','../documentacao/','evalCode',false)
    
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuSadriDadosTaylor','outputDir','../documentacao/','evalCode',false)

    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuSadriDadosAjuste','outputDir','../documentacao/','evalCode',false)

    % pneuPacejka
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuPacejkaFun','outputDir','../documentacao/','evalCode',false)

    % pneuPacejka estendido
    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuPacejkaEstFun','outputDir','../documentacao/','evalCode',false)

    close all % Fechando as figuras geradas
    clear all % Limpando o workspace
    publish('pneuPacejkaDados','outputDir','../documentacao/','evalCode',false)

%% Seletor

close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('seletor','outputDir','../documentacao/','evalCode',false)

%% Frame

close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('frame','outputDir','../documentacao/','evalCode',false)

%% Animação

close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('animacao','outputDir','../documentacao/','evalCode',false)

% Vetor
close all % Fechando as figuras geradas
clear all % Limpando o workspace
publish('vetor','outputDir','../documentacao/','evalCode',false)

close all % Fechando as figuras geradas
clear all % Limpando o workspace

clc % Limpando a command window

% Abrindo a página inicial da documentação
open ../documentacao/index.html

%% Ver também
%
% <index.html Início>
%