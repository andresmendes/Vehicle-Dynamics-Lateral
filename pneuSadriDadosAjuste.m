%% Dados pneu Sadri - Ajuste
% Determinação dos valores dos parâmetros para o modelo de pneu sadri. Os
% dados são obtidos a partir do ajuste da inclinação para ângulo igual a
% zero e força lateral máxima.
%
%% Dados
% Os parâmetros dos eixos dianteiros e traseiros visam ser equivalentes a
% um modelo de <pneuPacejkaFun.m pneuPacejka>. Isto é feito impondo as
% seguintes características:
%
% * Mesmo coeficiente de rigidez de curva para angulos de deriva pequenos
% * Mesma força lateral máxima
%
% Como o modelo aqui é dado por:
%
% $$ F_y = k_1 \alpha  - k_2\alpha^3 $$
%
% A derivada de $ F_y $ com relação a $\alpha$:
%
% $$ \frac{dF_y}{d \alpha} = k_1 - 3 k_2 \alpha^2 $$
%
% Logo, para $\alpha = 0$:
%
% $$ \left. \frac{dF_y}{d \alpha} \right|_{\alpha = 0} = k_1 $$
%
% Portanto, $k_1$ é o próprio coeficiente de rigidez de curva para pequenos
% ângulos.
%
% Como o valor de máxima força lateral $F_{ymax}$ ocorre no ângulo em que
% $\frac{dF_y}{d \alpha} = 0$ temos que este ângulo é dado por:
%
% $$ \alpha_{max} = \sqrt{\frac{k_1}{3 k_2}}$$
%
% Manipulando as equações, o valor de $k_2$ que fornece a mesma força
% lateral máxima no modelo de Sadri é dado por:
%
% $$k_2 = (4*k1^3)/(27*FyMax^2) $$
% 
% Onde $FyMax$ é o valor da força lateral máxima do modelo em relação ao
% qual se deseja obter a equivalência, neste caso, <pneuPacejkaFun.html
% pneuPacejka>
% 
%% Modelo de referência
% O modelo de referência usado é o modelo de pneu Pacejka que tem os
% seguintes dados:

pneuPacejkaDados

%% Código e dados
% Código dos dados:

FyMaxF = muy*FzF; % Força lateral máxima
FyMaxR = muy*FzR; % Força lateral máxima

CfaF = c1F*c2F*Fz0*sin(2*atan(FzF/(c2F*Fz0))); % Cfa em função de Fz
CfaR = c1R*c2R*Fz0*sin(2*atan(FzR/(c2R*Fz0))); % Cfa em função de Fz

Cfa0F = c1F*c2F*Fz0*sin(2*atan(Fz0/(c2F*Fz0))); % Cfa para Fz0
Cfa0R = c1R*c2R*Fz0*sin(2*atan(Fz0/(c2R*Fz0))); % Cfa para Fz0

Dy0 = muy0*Fz0;

By0F = Cfa0F/(CyF*Dy0); % Stiffness factor
By0R = Cfa0R/(CyR*Dy0); % Stiffness factor

% Modelo Sadri Equivalente
k1F = CyF*By0F*Dy0;
k2F = (4*k1F^3)/(27*FyMaxF^2);
k1R = CyR*By0R*Dy0;
k2R = (4*k1R^3)/(27*FyMaxR^2);

pneuDadosFrente = [k1F k2F];
pneuDadosTras = [k1R k2R];

%% Ver também
%
% <index.html Início> | <pneuDoc.html Modelo de pneu>
%