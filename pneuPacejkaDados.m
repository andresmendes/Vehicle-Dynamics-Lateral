%% Dados pneu Pacejka
% Determinação dos valores dos parâmetros para o modelo de pneu Pacejka.
%
% Os valores do pneu dianteiro e traseiro são obtidos separadamente.
%
%% Código e dados
% Código dos dados:

% Condições nominais
Fz0 = 2.4985e+04;    % Carga vertical nominal
muy0 = 0.8;          % Coeficiente de atrito nominal
% Condições de operação
FzF = Fz0;           % Eixo Dianteiro
FzR = Fz0;           % Eixo Traseiro
muy = muy0;          % Coeficiente de atrito de operação
% Coeficientes experimentais do pneu dianteiro
CyF = 1.5;
EyF = -2;
c1F = 3.5899;
c2F = 1.33;
% Coeficientes experimentaisdo pneu traseiro
CyR = 1.5;
EyR = -2;
c1R = 3.5899;
c2R = 1.33;

% Vetores de dados na dianteira e traseira
pneuDadosFrente = [Fz0 muy0 CyF EyF c1F c2F FzF muy];
pneuDadosTras = [Fz0 muy0 CyR EyR c1R c2R FzR muy];

%% Ver também
%
% <index.html Início> | <pneuDoc.html Modelo de pneu>
%