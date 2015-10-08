%% Dinâmica Veicular
% Esta é a documentação do repositório Dinamica-Veicular. Para download dos
% arquivos acessar: <https://github.com/andresmendes/Dinamica-Veicular>
%
%% Organização
% Este repositório é organizado em três camadas (ESTUDOS, MODELOS e DADOS)
% com mostrado na figura abaixo.
%%
%
% <<fluxograma_01.png>>
%
% A seguir cada camada é apresentada com os scripts disponíveis e os links
% para maiores informações.
%
%% Estudos
% Os estudos são os scripts que usam os modelos e seus respectivos dados
% para buscar alguma informação à respeito do comportamento dinâmico dos
% veículos. Estudos necessitam de modelos e dados. Lista dos estudos:
%
% <html> <table border=1 width="97%"> 
% <tr> <td width="30%"> <b> Estudo </b></td> <td width="70%"> <b> Descrição </b></td> </tr>
% <tr> <td width="30%"> 1. <a href="estudoSimples.html">Simples</a> </td> <td width="70%">Integração do modelo para esterçamento nulo e condições inicais diferentes de zero.</td> </tr>
% <tr> <td width="30%"> 2. <a href="estudoComparacaoDelta14.html">Comparação - DELTA = 14</a></td> <td width="70%">Comparação entre os modelos de pneu para diferentes modelos de veículo. Condições iniciais nulas e esterçamento igual a 14 graus.</td> </tr>
% <tr> <td width="30%"> 3. <a href="estudoComparacaoDelta45.html ">Comparação - DELTA = 45</a></td> <td width="70%"> Comparação entre os modelos de pneu para diferentes modelos de veículo. Condições iniciais nulas e esterçamento igual a 45 graus.</td> </tr>
% <tr> <td width="30%"> 4. <a href="estudoComparacaoDpsi2.html">Comparação - dPSI = 2</a></td> <td width="70%"> Comparação entre os modelos de pneu para diferentes modelos de veículo. Esterçamento nulo e condições iniciais dadas por: dPSI0 = 2 rad/s e ALPHAT0 = 0 rad.</td> </tr>
% <tr> <td width="30%"> 5. <a href="estudoComparacaoDpsi3com.html">Comparação - dPSI = 3 com Sadri</a></td> <td width="70%"> Com modelo Sadri. Comparação entre os modelos de pneu para diferentes modelos de veículo. Esterçamento nulo e condições iniciais dadas por: dPSI0 = 3 rad/s e ALPHAT0 = 0 rad.</td> </tr>
% <tr> <td width="30%"> 6. <a href="estudoComparacaoDpsi3sem.html">Comparação - dPSI = 3 sem Sadri</a></td> <td width="70%">Sem modelo Sadri. Comparação entre os modelos de pneu para diferentes modelos de veículo. Esterçamento nulo e condições iniciais dadas por: dPSI0 = 3 rad/s e ALPHAT0 = 0 rad.</td> </tr>
% <tr> <td width="30%"> 7. <a href="estudoComparacaoDpsi3est.html">Comparação - dPSI = 3 estendido</a></td> <td width="70%">Comparação entre os modelos de pneu <a href="pneuPacejkaFun.html"> Pacejka</a> e <a href="pneuPacejkaEstFun.html"> Pacejka estendido</a>. Esterçamento nulo e condições iniciais dadas por: dPSI0 = 3 rad/s e ALPHAT0 = 0 rad.</td> </tr>
% <tr> <td width="30%"> 8. <a href="estudoComparacaoDpsi3veiculo.html">Comparação - dPSI = 3 veiculo</a></td> <td width="70%">Comparação entre os modelos de veículo com modelo de pneu <a href="pneuPacejkaEstFun.html"> Pacejka estendido</a>. Esterçamento nulo e condições iniciais dadas por: dPSI0 = 3 rad/s e ALPHAT0 = 0 rad.</td> </tr>
% </table> </html>
%
%% Modelos
% Os modelos são usados pelos sripts de estudo e necessitam de dados.
%
% *Modelo de veículo*
%
% Os modelos de veículo são as equações diferenciais que descrevem a
% dinâmica do veículo. Os modelos disponíveis são:
%
% # <veiculoLinear2gdl.html Linear com 2 GDL>
% # <veiculoNaoLinear2gdl.html Não linear com 2 GDL>
% # <veiculoNaoLinear3gdl.html Não linear com 3 GDL>
%
% Para maiores detalhes ver: <veiculoDoc.html Modelo de veículo>.
%
% *Modelos de pneu*
%
% Os modelos de pneu descrevem a curva característica do pneu. Os modelos
% disponíveis são:
%
% # <pneuLinearFun.html Pneu linear>
% # <pneuSadriFun.html Pneu Sadri>
% # <pneuPacejkaFun.html Pneu Pacejka>
% # <pneuPacejkaEstFun.html Pneu Pacejka Estendido>
%
% Para maiores detalhes ver: <pneuDoc.html Modelo de pneu>.
%
%% Dados
% Os dados são usados pelos scripts de modelos que por sua vez são usados
% pelos scripts de estudos.
%
% *Dados de veículo*
%
% Lista de dados para modelos de veículo:
%
% # <veiculoDadosDelta0.html Dados delta 0>
% # <veiculoDadosDelta14.html Dados delta 14>
% # <veiculoDadosDelta45.html Dados delta 45>
%
% Para os modelos de veículo listados acima todos os dados de veículo são
% compatíveis.
%
% *Dados do pneu*
%
% Lista dos dados para modelos de pneu de acordo com a compatibilidade:
%
% <html> <table border=1 width="97%"> 
% <tr> <td width="25%"> <b> Modelos de pneu </b></td> <td width="25%"> <b> Dados compatíveis </b></td> <td width="50%"> <b> Descrição </b> </td></tr>
% <tr> <td width="25%"> 1. <a href="pneuLinearFun.html">Pneu linear</a> </td> <td width="25%"> 1. <a href="pneuLinearDados.html">Dados para pneu linear</a> </td> <td width="50%"> Modelo linear de pneu. </td></tr>
% <tr> <td width="25%"> 2. <a href="pneuSadriFun.html">Pneu Sadri</a></td> <td width="25%"> 2. <a href="pneuSadriDadosTaylor.html">Dados para pneu Sadri Taylor</a> <br> 3. <a href="pneuSadriDadosAjuste.html">Dados para pneu Sadri Ajuste</a> </td> <td width="50%"> Modelo não linear de terceira ordem. </td></tr>
% <tr> <td width="25%"> 3. <a href="pneuPacejkaFun.html">Pneu Pacejka</a></td> <td width="25%"> 4. <a href="pneuPacejkaDados.html">Dados para pneu Pacejka</a> </td> <td width="50%"> Modelo não linear. Equação semi empírica com coeficientes experimentais </td></tr>
% <tr> <td width="25%"> 4. <a href="pneuPacejkaEstFun.html">Pneu Pacejka estendido</a></td> <td width="25%"> 4. <a href="pneuPacejkaDados.html">Dados para pneu Pacejka</a> </td> <td width="50%"> Modelo não linear. Equação semi empírica com coeficientes experimentais. Tratamento do ângulo de deriva. </td></tr>
% </table> </html>
%
%% Documentação
% Para gerar toda a documentação: <docDin.html docDin.m>
%
%% Extra
% Para gerar a figura de fluxograma: <fluxograma.html fluxograma.m>
% Para gerar a animacao: <animacao.html animacao.m>
% Para gerar os vetores na animação: <vetor.html vetor.m>
%
%% Encoding
% Todos os arquivos deste repositório utilizam o encoding: windows-1252 (via
% Sublime3). Logo, o encoding do Matlab deve ser o mesmo. Para isto, utilize
% o seguinte código na linha de comando:

% Verifica o encoding:
slCharacterEncoding() 
% Modifica o encoding para 'windows-1252'
slCharacterEncoding('Windows-1252')

%%
% Para rodar os códigos em sistemas operacionais Windows (Testado em
% Windows(7)/Matlab 2014a): sem problemas pois encoding windows-1252 é o
% padrão (Verificar com os comandos acima). Porém o texto no editor ou em
% figuras pode apresentar erros quando executados em sistemas operacionais
% Linux.
%
% Para rodar em Linux (Testado em Ubuntu14.04)/Matlab 2013a): com problemas
% pois UTF-8 é o padrão (Verificar). Logo:
%
% * Usar na linha de comando: 'slCharacterEncoding('Windows-1252')'
% * Mesmo assim editor não funciona: não exibe os caracteres corretamente
% (Usar
% 	Sublime-Text por exemplo)
% * Os gráficos ficam direito.
%