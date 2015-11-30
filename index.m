%% Dinâmica Veicular
% Este repositório fornece as classes de simulação, modelo de pneu, modelo de veículo e geração de gráficos.
%
%% Instruções
% Para a utilização do pacote é necessário realizar os seguintes passos:
%
% * Baixar o pacote Dinamica-Veicular clicando em "Download ZIP" na página <https://github.com/andresmendes/Dinamica-Veicular https://github.com/andresmendes/Dinamica-Veicular>.
% * Salvar o pacote num _path_ do MATLAB(R) ou adicionar o diretório na lista de _paths_. Mais detalhes em <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
% * Importar o pacote inteiro digitando no _command window_:
%

import DinamicaVeicular.*

%% Simulação
% A simulação consiste na integração numérica do sistema.
%
% Para Rodar simulação padrão:
%

Simula

%%
% Para maiores detalhes ver: <Simula.html Simula>
%
% Para rodar simulação diferente da padrão: ver <Template.html Template.m>.
%
%% Pneu
% Relação entre a força lateral e o ângulo de deriva. <Pneu.html Pneu (Abstract)>
%
% <<CurvaCaracteristica.png>>
%
% Modelos de pneu:
%
% * <PneuLinear.html Pneu Linar>
% * <PneuPolinomial.html Pneu Polinomial>
% * <PneuPacejka.html Pneu Pacejka>
%
%% Veiculo
% Equação do movimento. <Veiculo.html Veiculo (Abstract)>
%
% <<modelo.png>>
%
% Modelos de veículo:
%
% * <VeiculoArticuladoNaoLinear4GDL.html Veiculo Articulado Linear 4 GDL> (Pendente)
% * <VeiculoArticuladoNaoLinear4GDL.html Veiculo Articulado Não Linear 4 GDL>
%
%% Gráficos
% Evolução dos estados, planos e espaço de fase, frame e animação. <Graficos.html Graficos>
