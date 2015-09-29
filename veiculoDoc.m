%% Modelo de veículo
% Dinâmica do veículo.
%
%% Sintaxe
% |dx =
% _veiculoFun_(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDados)|
%% Argumentos
% Lista de entradas da função:
%
% <html> <table border=1 width="97%"> 
% <tr> <td width="30%"><tt>t</tt></td> <td width="70%">Tempo.</td> </tr> 
% <tr> <td> <tt> x </tt></td> <td>Estados.</td> </tr>
% <tr> <td> <tt> pneuFun </tt></td> <td>Handle da função do modelo de pneu. Ver <a href="pneuDoc.html">Modelo pneu</a>. </td></tr>
% <tr> <td> <tt> pneuDadosFrente </tt></td> <td>Vetor com os dados do pneu dianteiro. Ver <a href="pneuDoc.html">Modelo pneu</a>.</td> </tr>
% <tr> <td> <tt> pneuDadosTras </tt></td> <td>Vetor com os dados do pneu traseiro. Ver <a href="pneuDoc.html">Modelo pneu</a>.</td> </tr>
% <tr> <td> <tt> veiculoDados</tt></td> <td>Vetor com os dados do veículo: <tt>[m I a b v DELTA]</tt></td> </tr>
% </table> </html>
% 
% Lista de saídas da função:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>dx</tt></td> <td width="70%">Derivada dos estados.</td></tr>
% </table> </html>
% 
%% veiculoFun
% As opções de modelos de veículos são aprentadas na tabela abaixo:
%
% <html> <table border=1 width="97%"> 
% <tr><td width="30%"> <b> veiculoFun </b></td><td width="70%"> <b>Modelo</b></td></tr> 
% <tr> <td> <a href="veiculoLinear2gdl.html"><tt>veiculoLinear2gdl</tt></a> </td> <td>Veículo linear com 2 GDL</td></tr>
% <tr> <td> <a href="veiculoNaoLinear2gdl.html"><tt>veiculoNaoLinear2gdl</tt></a> </td> <td> Veículo não linear com 2 GDL</td></tr>
% <tr> <td> <a href="veiculoNaoLinear3gdl.html"><tt>veiculoNaoLinear3gdl</tt></a> </td> <td> Veículo não linear com 3 GDL</td></tr>
% <tr> <td> <a href="veiculoNaoLinear3gdlEst.html"><tt>veiculoNaoLinear3gdlEst</tt></a> </td> <td> Veículo não linear com 3 GDL com tratamento do ângulo de deriva dos pneus</td></tr>
% </table> </html>
%
%% Exemplo - Simulação simples
% Ver o script <simulacao.html simulacao>.
%
%% Exemplo - Comparação
% Ver o script <comparacao.html comparacao>.
%
%% Exemplo - Região de convergência
% Ver o script <regiao.html regiao>.
%
%% Ver também
%
% <inicio.html Início> 
%