%% Gerador de documentação
%
% index
publish('index','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
% Simula
publish('+DinamicaVeicular/@Simula/Simula','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
% Template
publish('Template','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
% Pneu
publish('+DinamicaVeicular/@Pneu/Pneu','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuLinear/PneuLinear','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuPolinomial/PneuPolinomial','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuPacejka/PneuPacejka','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
% Veículo
publish('+DinamicaVeicular/@Veiculo/Veiculo','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@VeiculoArticuladoNaoLinear4GDL/VeiculoArticuladoNaoLinear4GDL','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
% Graficos
publish('+DinamicaVeicular/@Graficos/Graficos','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);

close all   % Fechando as figuras geradas
clear all   % Limpando o workspace
clc         % Limpando o command window
