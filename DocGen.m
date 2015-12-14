%% Doc Generator
%

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% index
publish('index','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);

% Templates
publish('TemplateSimples','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',true);
    clear all                   % Clear workspace
    close all                   % Closing figures
    clc                         % Clear command window
publish('TemplateArticulado','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',true);
    clear all                   % Clear workspace
    close all                   % Closing figures
    clc                         % Clear command window

% Tire model
publish('+DinamicaVeicular/@Pneu/Pneu','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuLinear/PneuLinear','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuPolinomial/PneuPolinomial','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@PneuPacejka1989/PneuPacejka1989','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
publish('ComparacaoPneu','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',true);
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% Vehicle model
publish('+DinamicaVeicular/@VeiculoArticulado/VeiculoArticulado','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@VeiculoArticuladoNaoLinear4GDL/VeiculoArticuladoNaoLinear4GDL','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
publish('+DinamicaVeicular/@VeiculoSimples/VeiculoSimples','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
    publish('+DinamicaVeicular/@VeiculoSimplesNaoLinear3GDL/VeiculoSimplesNaoLinear3GDL','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);

% Graphics
publish('+DinamicaVeicular/@Graficos/Graficos','outputDir','../Dinamica-Veicular-Documentacao/','evalCode',false);
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window
