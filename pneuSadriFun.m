%% Pneu Sadri
% Relação não linear entre a força lateral e o ângulo de deriva.
%
%% Sintaxe
% |Fy = pneuSadriFun(deriva,pneuDados)|
%
%% Argumentos
% Lista de entradas da função:
%
% <html> <table border=1 width="97%"> <tr> <td
% width="30%"><tt>deriva</tt></td> <td width="70%">Ângulo de deriva do
% pneu. Ângulo formado entre o vetor velocidade e o plano longitudinal do
% pneu.</td> </tr> <tr> <td><tt>pneuDados</tt></td> <td>Vetor com os dados
% do pneu: <tt>[k1 k2]</tt></td> </tr> </table> </html>
% 
% Lista de saídas da função:
%
% <html> <table border=1 width="97%"> <tr> <td width="30%"><tt>Fy</tt></td>
% <td width="70%">Força lateral do pneu.</td></tr></table> </html>
%
%% Dados
% Dados disponíveis para este modelo:
%
% * <pneuSadriDadosTaylor.html pneuSadriDadosTaylor>
% * <pneuSadriDadosAjuste.html pneuSadriDadosAjuste>
%
%% Modelo
%
% *Equacionamento*
%
% A equação que descreve este modelo é dada por:
%
% $$ F_y = k_1 \alpha  - k_2\alpha^3 $$
%
% Onde $F_y$ é a força lateral e $\alpha$ é o ângulo de deriva. $k_1$ e
% $k_2$ são constantes do modelo.
%
% *Hipóteses*
%
% * Relação não linear.
% * Válido apenas até o ângulos de deriva que fornece a máxima força
% lateral (Saturação do pneu).
%
%% Código
% Código da função:

function Fy = pneuSadriFun(deriva,pneuDados)
% Parâmetros do pneu
k1 = pneuDados(1);
k2 = pneuDados(2);

alpha = deriva; % Ângulo de deriva [rad]

Fy = -(k1*alpha-k2*alpha.^3); % Força lateral [N]
end

%% Exemplos
% Ver: <pneuDoc.html Modelo de pneu>
%
%% Ver também
%
% <index.html Início> | <pneuDoc.html Modelo de pneu> |
% 