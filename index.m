%% Vehicle Dynamics
% Este reposit�rio tem como objetivo servir de biblioteca de fun��es destinadas ao estudo de din�mica veicular.
%
%% Instructions
% Para a utiliza��o do pacote � necess�rio realizar os seguintes passos:
%
% * Baixar o pacote Dinamica-Veicular clicando em "Download ZIP" na p�gina <https://github.com/andresmendes/Vehicle-Dynamics https://github.com/andresmendes/Dinamica-Veicular>.
% * Salvar o pacote (pasta "+DinamicaVeicular") num _path_ do MATLAB(R) ou adicionar o caminho na lista de _paths_. Mais detalhes em <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
% * Importar o pacote inteiro com o comando:
%

import DinamicaVeicular.*

%% Description
% Este reposit�rio � destinado � organizar as fun��es necess�rias para a integra��o num�rica das equa��es de movimento de ve�culos e ilustrar o resultado. A estrutura m�nima de simula��o se encontra no fluxograma abaixo. Com os par�metros de integra��o e condi��es iniciais o modelo do sistema pode ser integrado para a obten��o da evolu��o dos estados em fun��o do tempo. O modelo do sistema consiste na combina��o do modelo de ve�culo com o modelo de pneu e seus respectivos par�metros. Por fim, � poss�vel gerar os gr�ficos � partir dos resultados. Portanto, as fun��es dispon�veis neste reposit�rio procuram realizar as tarefas representadas pelos blocos de *modelo de ve�culo*, *modelo de pneu* e *gr�ficos*.
%
% <<ilustracoes/fluxograma.svg>>
%
%% Templates
% Os templates realizam uma simula��o de ve�culo utilizando os elementos descritos no fluxograma acima.
% Para auxiliar os primeiros passos na utiliza��o das fun��es est�o dispon�veis dois templates:
%
% * Simula��o de ve�culo simples: <TemplateSimples.html TemplateSimples.m>
% * Simula��o de ve�culo articulado: <TemplateArticulado.html TemplateArticulado.m>
%
%% Tire model
% Rela��o entre a for�a lateral e o �ngulo de deriva.
%
% Modelos de pneu:
%
% * <PneuLinear.html Pneu Linar>
% * <PneuPolinomial.html Pneu Polinomial>
% * <PneuPacejka1989.html Pneu Pacejka>
%
% Extra:
%
% * <ComparacaoPneu.html Compara��o pneu>
%
% Outros: <Pneu.html Pneu (Abstract)>
%
%% Vehicle model
% Fun��o com a equa��o de estados do modelo.
%
% Modelo de ve�culo:
%
% * <VeiculoSimplesLinear2GDL.html Veiculo Simples Linear 2 GDL> (Pendente)
% * <VeiculoSimplesNaoLinear3GDL.html Veiculo Simples N�o Linear 3 GDL>
% * <VeiculoArticuladoLinear4GDL.html Veiculo Articulado Linear 3 GDL> (Pendente)
% * <VeiculoArticuladoNaoLinear4GDL.html Veiculo Articulado N�o Linear 4 GDL>
%
% Outros: <VeiculoSimples.html Veiculo simples (Abstract)> | <VeiculoArticulado.html Veiculo articulado (Abstract)>
%
%% Graphics
% Fun��es para a gera��o de ilustra��es: <Graficos.html Graficos>.
%
%% See Also
%
% <https://github.com/andresmendes/Dinamica-Veicular https://github.com/andresmendes/Dinamica-Veicular>
%
