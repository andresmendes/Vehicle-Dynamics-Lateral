%% Dinâmica Veicular
% Este repositório tem como objetivo servir de biblioteca de funções destinadas ao estudo de dinâmica veicular.
%
%% Instructions
% Para a utilização do pacote é necessário realizar os seguintes passos:
%
% * Baixar o pacote Dinamica-Veicular clicando em "Download ZIP" na página <https://github.com/andresmendes/Dinamica-Veicular https://github.com/andresmendes/Dinamica-Veicular>.
% * Salvar o pacote (pasta "+DinamicaVeicular") num _path_ do MATLAB(R) ou adicionar o caminho na lista de _paths_. Mais detalhes em <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
% * Importar o pacote inteiro com o comando:
%

import DinamicaVeicular.*

%% Description
% Este repositório é destinado à organizar as funções necessárias para a integração numérica das equações de movimento de veículos e ilustrar o resultado. A estrutura mínima de simulação se encontra no fluxograma abaixo. Com os parâmetros de integração e condições iniciais o modelo do sistema pode ser integrado para a obtenção da evolução dos estados em função do tempo. O modelo do sistema consiste na combinação do modelo de veículo com o modelo de pneu e seus respectivos parâmetros. Por fim, é possível gerar os gráficos à partir dos resultados. Portanto, as funções disponíveis neste repositório procuram realizar as tarefas representadas pelos blocos de *modelo de veículo*, *modelo de pneu* e *gráficos*.
%
% <<ilustracoes/fluxograma.svg>>
%
%% Templates
% Os templates realizam uma simulação de veículo utilizando os elementos descritos no fluxograma acima.
% Para auxiliar os primeiros passos na utilização das funções estão disponíveis dois templates:
%
% * Simulação de veículo simples: <TemplateSimples.html TemplateSimples.m>
% * Simulação de veículo articulado: <TemplateArticulado.html TemplateArticulado.m>
%
%% Tire model
% Relação entre a força lateral e o ângulo de deriva.
%
% Modelos de pneu:
%
% * <PneuLinear.html Pneu Linar>
% * <PneuPolinomial.html Pneu Polinomial>
% * <PneuPacejka1989.html Pneu Pacejka>
%
% Extra:
%
% * <ComparacaoPneu.html Comparação pneu>
%
% Outros: <Pneu.html Pneu (Abstract)>
%
%% Vehicle model
% Função com a equação de estados do modelo.
%
% Modelo de veículo:
%
% * <VeiculoSimplesLinear2GDL.html Veiculo Simples Linear 2 GDL> (Pendente)
% * <VeiculoSimplesNaoLinear3GDL.html Veiculo Simples Não Linear 3 GDL>
% * <VeiculoArticuladoLinear4GDL.html Veiculo Articulado Linear 3 GDL> (Pendente)
% * <VeiculoArticuladoNaoLinear4GDL.html Veiculo Articulado Não Linear 4 GDL>
%
% Outros: <VeiculoSimples.html Veiculo simples (Abstract)> | <VeiculoArticulado.html Veiculo articulado (Abstract)>
%
%% Graphics
% Funções para a geração de ilustrações: <Graficos.html Graficos>.
