%% Veículo não linear com 3 GDL
% Modelo bicicleta não linear com 3 graus de liberdade
%
%% Sintaxe
% |dx =
% veiculoNaoLinear3gdl(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDados)|
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
%% Código
% Código da função:

function dx = veiculoNaoLinear3gdl(t,x,pneuFun,pneuDadosFrente,pneuDadosTras,veiculoDados)

% Dados do veículo
m = veiculoDados(1); % massa do veiculo [kg]
I = veiculoDados(2); % momento de inercia [kg]
a = veiculoDados(3); % distancia do eixo dianteiro ao centro de massa [m]
b = veiculoDados(4); % distancia do eixo dianteiro ao centro de massa [m]
% Pelo amor de deus! isso devia ta dando mta cagada:
%v = veiculoDados(5); % módulo da velocidade do centro de massa [m/s]

DELTA = veiculoDados(6);

% Estados
dPSI = x(1);
ALPHAT = x(2);
v = x(6);
PSI = x(3);


% Angulos de deriva não linear
ALPHAF = atan2((v*sin(ALPHAT) + a*dPSI),(v*cos(ALPHAT))) - DELTA; % Dianteiro
ALPHAR = atan2((v*sin(ALPHAT) - b*dPSI),(v*cos(ALPHAT)));         % Traseiro

% Forças longitudinais
FxF = 0;
FxR = 0;

% Curva característica
FyF = pneuFun(ALPHAF,pneuDadosFrente);
FyR = pneuFun(ALPHAR,pneuDadosTras);

% Equações de estado
dx(1,1) = (FyF*a*cos(DELTA) - FyR*b + FxF*a*sin(DELTA))/I;
dx(2,1) = (FyR + FyF*cos(DELTA) + FxF*sin(DELTA) - m*(dPSI*v*cos(ALPHAT) + (sin(ALPHAT)*(FxR + 	FxF*cos(DELTA) - ...
	FyF*sin(DELTA) + dPSI*m*v*sin(ALPHAT)))/(m*cos(ALPHAT))))/(m*(v*cos(ALPHAT) + (v*sin(ALPHAT)^2)/cos(ALPHAT)));
dx(6,1) = (FxR*cos(ALPHAT) + FyR*sin(ALPHAT) - FyF*cos(ALPHAT)*sin(DELTA) + FyF*cos(DELTA)*sin(ALPHAT) + ...
	FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA))/(m*cos(ALPHAT)^2 + m*sin(ALPHAT)^2);

% Obtenção da orientação
dx(3,1) = dPSI; % dPSI

% Equações adicionais para o posicionamento (Não necessárias para a dinâmica em guinada)
dx(4,1) = v*cos(ALPHAT + PSI); % X
dx(5,1) = v*sin(ALPHAT + PSI); % Y
end

%% Ver também
%
% <index.html Início> | <veiculoDoc.html Modelo de veículo>
%