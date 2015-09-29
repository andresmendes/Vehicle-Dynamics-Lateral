%% Pneu linear
% Relação linear entre a força lateral e o ângulo de deriva
%
%% Sintaxe
% |Fy = pneuLinearFun(deriva,pneuDados)|
%
%% Argumentos
% Lista de entradas da função:
%
% <html> <table border=1 width="97%"> <tr> <td
% width="30%"><tt>deriva</tt></td> <td width="70%">Ângulo de deriva do
% pneu. Ângulo formado entre o vetor velocidade e o plano longitudinal do
% pneu.</td> </tr> <tr> <td><tt>pneuDados</tt></td> <td>Vetor com os dados
% do pneu: <tt>C</tt> </td> </tr> </table> </html>
% 
% Lista de saídas da função:
%
% <html> <table border=1 width="97%"> <tr> <td width="30%"><tt>Fy</tt></td>
% <td width="70%">Força lateral do pneu.</td></tr></table> </html>
% 
%% Dados
% Dados disponíveis para este modelo:
%
% * <pneuLinearDados.html pneuLinearDados> (Linear)
%
%% Modelo
%
% *Equacionamento*
%
% A equação que descreve este modelo é dada por:
%
% $$ F_y = C \alpha $$
%
% Onde $F_y$ é a força lateral, $C$ é o coeficiente de rigidez de curva e
% $\alpha$ é o ângulo de deriva.
%
% *Hipóteses*
%
% * Relação linear.
% * Válido apenas para pequenos ângulos de deriva.
%
%% Código
% Código da função:

function Fy = pneuLinearFun(deriva,pneuDados)
C = pneuDados(1);   % Coeficiente de rigidez de curva [N/rad]
alpha = deriva;     % Ângulo de deriva [rad]
Fy = -C*alpha;      % Força lateral [N]
end

%% Exemplos
% Ver: <pneuDoc.html Modelo de pneu>
%
%% Ver também
%
% <index.html Início> | <pneuDoc.html Modelo de pneu>
%