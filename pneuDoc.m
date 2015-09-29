%% Modelo de pneu
% Relação entre a força lateral e o ângulo de deriva.
%
%% Sintaxe
% |Fy = _pneuFun_(deriva,pneuDados)|
%
%% Argumentos
% Lista de entradas da função:
%
% <html> <table border=1 width="97%"> <tr> <td
% width="30%"><tt>deriva</tt></td> <td width="70%">Ângulo de deriva do
% pneu. Ângulo formado entre o vetor velocidade e o plano longitudinal do
% pneu.</td> </tr> <tr> <td><tt>pneuDados</tt></td> <td>Vetor com os dados
% do pneu.</td> </tr> </table> </html>
% 
% Lista de saídas da função:
%
% <html> <table border=1 width="97%"> <tr> <td width="30%"><tt>Fy</tt></td>
% <td width="70%">Força lateral do pneu.</td></tr></table> </html>
% 
%% Funções de pneu - pneuFun
% As opções de funções de pneu são aprentadas na tabela abaixo:
%
% <html> <table border=1 width="97%"> <tr><td width="30%"> <b> pneuFun
% </b></td><td width="35%"> <b>Modelo</b></td><td width="35%"><b>Argumentos
% de entrada</b></td></tr> <tr> <td> <a
% href="pneuLinearFun.html"><tt>pneuLinearFun</tt></a> </td> <td>Pneu
% linear</td><td> <tt>delta, pneuDados = [C]</tt> </td></tr><tr> <td> <a
% href="pneuSadriFun.html"><tt>pneuSadriFun</tt></a> </td> <td>Pneu
% Sadri</td><td> <tt>delta, pneuDados = [k1 k2]</tt> </td></tr><tr> <td> <a
% href="pneuPacejkaFun.html"><tt>pneuPacejkaFun</tt></a> </td> <td>Pneu
% pacejka</td><td> <tt>delta, pneuDados = [Fz0 muy0 Cy Ey c1 c2 Fz muy]</tt>
% </td></tr></table> </html>
%
%% Dados do pneu - pneuDados
% Os dados do pneu dependem do modelo adotado. Este repositório possui os
% seguintes dados de pneu para cada modelo:
%
% * <pneuLinearDados.html pneuLinearDados> (Linear)
% * <pneuSadriDadosTaylor.html pneuSadriDadosTaylor> (Sadri)
% * <pneuSadriDadosAjuste.html pneuSadriDadosAjuste> (Sadri)
% * <pneuPacejkaDados.html pneuPacejkaDados> (Pacejka)
%
% Estes modelos fornecem as informações dos parâmetros dos pneus para a
% determinação da força lateral exercida em função do ângulo de deriva.
%
%% Modelos
% O modelo de pneu relaciona a força lateral com o ângulo de deriva (Ângulo
% formado entre o vetor velocidade do centro do pneu com o plano
% longitudinal do pneu). A relação típica entre essas duas grandezas pode
% ser observada na figura abaixo. Além disso é possível verificar a
% definição do ângulo de deriva.
%
% <<curvaCaracteristica.png>>
%
% *Modelo Linear*
%
% Para simulações em que os ângulos de deriva sejam pequenos é possível
% aproximar a curva característica por uma relação linear dada por:
%
% $$ F_y = C \alpha $$
%
% Onde $F_y$ é a força lateral, $C$ é o coeficiente de rigidez de curva e
% $\alpha$ é o ângulo de deriva.
%
% Para maiores informações sobre o modelo linear ver: <pneuLinearFun.html
% pneuLinearFun>
%
% As opçoes de dados para este modelo são: <pneuLinearDados.html
% pneuLinearDados>
%
% *Modelo Sadri*
%
% Na tentativa de levar em consideração a não linearidade da curva
% característica muitos autores adotam o modelo de terceira ordem dado por:
%
% $$ F_y = k_1 \alpha  - k_2\alpha^3 $$
%
% Onde $k_1$ e $k_2$ são constantes do modelo.
%
% Para maiores informações sobre o modelo sadri ver: <pneuSadriFun.html
% pneuSadriFun>
%
% As opçoes de dados para este modelo são: <pneuSadriDadosTaylor.html
% pneuSadriDadosTaylor> e <pneuSadriDadosAjuste.html pneuSadriDadosAjuste>
%
% *Modelo Pacejka*
% 
% Uma outra alternativa para o modelar o pneu levando em consideração o seu
% comportamento não linear é através da versão simplificada da Magic
% Formula apresentada por Pacejka.
%
% $$ F_y = D \sin(C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha))))$$
%
% Onde $B$, $C$, $D$ e $E$ são coeficientes obtidos experimentalmente.
%
% Para maiores informações sobre o modelo Pacejka ver: <pneuPacejkaFun.html
% pneuPacejkaFun>
%
% As opçoes de dados para este modelo são: <pneuPacejkaDados.html
% pneuPacejkaDados>
%
%% Exemplo - Linear
% Ilustrando a curva característica do pneu para um modelo linear.

deriva = (0:0.7:25)*pi/180; % Ângulo de deriva [rad]

pneuLinearDados % Obtendo os dados do pneu linear (Dianteira e traseira)

pneuDados = pneuDadosFrente; % Atribuindo aos dados usados na geração da curva

FyLinear = pneuLinearFun(deriva,pneuDados);

figure(1)
box on
plot(deriva*180/pi,-FyLinear,'Color','r','Marker','o','MarkerFaceColor','r')
title('Modelo de pneu - Linear')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')

%% Exemplo - Sadri (Série de Taylor)
% Ilustrando a curva característica do pneu para um modelo sadri.

deriva = (0:0.7:25)*pi/180; % Ângulo de deriva [rad]

% Obtendo os dados do pneu Sadri a partir da expansão da Magic Formula em
% série de Taylor (Dianteira e traseira)
pneuSadriDadosTaylor 

pneuDados = pneuDadosFrente; % Atribuindo aos dados usados na geração da curva

FySadriST = pneuSadriFun(deriva,pneuDados);

figure(2)
box on
plot(deriva*180/pi,-FySadriST,'Color','c','Marker','*','MarkerFaceColor','c')
title('Modelo de pneu - Sadri (Série de Taylor)')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')

%% Exemplo - Sadri (Ajuste)
% Ilustrando a curva característica do pneu para um modelo sadri.

deriva = (0:0.7:25)*pi/180; % Ângulo de deriva [rad]

% Obtendo os dados do pneu Sadri a partir dos ajustes de inclinação e força
% lateral máxima (Dianteira e traseira)
pneuSadriDadosAjuste

pneuDados = pneuDadosFrente; % Atribuindo aos dados usados na geração da curva

FySadriAj = pneuSadriFun(deriva,pneuDados);

figure(3)
box on
plot(deriva*180/pi,-FySadriAj,'Color','g','Marker','s','MarkerFaceColor','g')
title('Modelo de pneu - Sadri (Ajuste)')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')

%% Exemplo - Pacejka
% Ilustrando a curva característica do pneu para um modelo pacejka.

deriva = (0:0.7:25)*pi/180; % Ângulo de deriva [rad]

pneuPacejkaDados % Obtendo os dados do pneu Pacejka (Dianteira e traseira)

pneuDados = pneuDadosFrente;

FyPacejka = pneuPacejkaFun(deriva,pneuDados);

figure(4)
box on
plot(deriva*180/pi,-FyPacejka,'Color','b','Marker','d','MarkerFaceColor','b')
title('Modelo de pneu - Pacejka')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')

%% Exemplo - Comparação
% Comparação dos modelos para parâmetros equivalentes.

figure(5)
box on
hold on
plot(deriva*180/pi,-FyLinear,'Color','r','Marker','o','MarkerFaceColor','r')
plot(deriva*180/pi,-FySadriST,'Color','c','Marker','*','MarkerFaceColor','c')
plot(deriva*180/pi,-FySadriAj,'Color','g','Marker','s','MarkerFaceColor','g')
plot(deriva*180/pi,-FyPacejka,'Color','b','Marker','d','MarkerFaceColor','b')
title('Comparação - Modelo de pneu')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')
legend('Linear','Sadri - Taylor','Sadri - Ajuste','Pacejka','Location','NorthWest')

%% Exemplo - Comparação (Grandes ângulos)
% *Pacejka*
%

deriva = (0:3:180)*pi/180; % Ângulo de deriva [rad]

pneuPacejkaDados % Obtendo os dados do pneu Pacejka (Dianteira e traseira)

pneuDados = pneuDadosFrente;

FyPacejka = pneuPacejkaFun(deriva,pneuDados);

%%
%
% *Pacejka estendido*
%
deriva = (0:3:180)*pi/180; % Ângulo de deriva [rad]

pneuPacejkaDados % Obtendo os dados do pneu Pacejka (Dianteira e traseira)

pneuDados = pneuDadosFrente;

FyPacejkaEst = pneuPacejkaEstFun(deriva,pneuDados);

figure(6)
box on
hold on
plot(deriva*180/pi,-FyPacejka,'Color','b','Marker','d','MarkerFaceColor','b')
plot(deriva*180/pi,-FyPacejkaEst,'Color','m','Marker','+','MarkerFaceColor','m')
title('Comparação - Grandes ângulos')
ylabel('Força lateral [N]')
xlabel('Ângulo de deriva [grau]')
legend('Pacejka','Pacejka estendido','Location','South')


%% Ver também
%
% <index.html Início> | <pneuLinearFun.html Pneu linear> |
% <pneuSadriFun.html Pneu sadri> | <pneuPacejkaFun.html Pneu pacejka>
%