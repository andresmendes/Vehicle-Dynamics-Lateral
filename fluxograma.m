%% Fluxograma
% Este script gera a figura que ilustra o fluxograma das categorias dos
% scripts do repositório

f = figure(1);

estudos = annotation('textbox',[.35 .8, .3, .1], 'String', 'Estudos');
    set(estudos,'HorizontalAlignment','Center','FontSize',20)
modelos = annotation('textbox',[.35 .5, .3, .1], 'String', 'Modelos');
    set(modelos,'HorizontalAlignment','Center','FontSize',20)
dados = annotation('textbox',[.35 .2, .3, .1], 'String', 'Dados');
    set(dados,'HorizontalAlignment','Center','FontSize',20)

flecha1 = annotation('arrow',[.5 .5], [.78, .62]);
flecha2 = annotation('arrow',[.5 .5], [.48, .32]);