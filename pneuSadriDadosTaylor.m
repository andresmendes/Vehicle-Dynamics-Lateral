%% Dados pneu Sadri - Série de Taylor
% Determinação dos valores dos parâmetros para o modelo de pneu sadri. Os
% dados são obtidos a partir da expansão da Magic Formula em série de
% Taylor.
%
%% Dados
% Os dados apresentados aqui são obtidos a partir da expansão em série de
% Taylor do modelo de pneu <pneuPacejkaFun.m pneuPacejka>.
%
% A expandindo em série de Taylor temos que:
% 
% $$ \arctan(\delta) = \delta - \frac{\delta^3}{3} ... $$
%
% $$ \sin(\delta) = \delta - \frac{\delta^3}{3!} ... $$
%
% Logo, a Magic Formula:
%
% $$ F_y = D \sin(C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha))))$$
%
% Pode ser escrita como (Truncando a série de Taylor no termo de terceira
% ordem):
% 
% $$ F_y \approx D (z - \frac{1}{6} z^3)$$
%
% Onde:
%
% $$ z = C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha)))$$
%
% Que também pode ser expandida como:
%
% $$ z \approx D (y - \frac{1}{3} y^3)$$
%
% Onde:
%
% $$ y = (B \alpha - E (B \alpha - \arctan(B \alpha))) $$
%
% Que também pode ser expandida como:
% 
% $$ y \approx B \alpha - E B \alpha + E (x - \frac{1}{3} x^3)$$
%
% Onde:
%
% $$ x = B \alpha $$
%
% Ignorando os termos de ordem superior a três a Magic Formula expandida em
% série de Taylor pode ser aproximada por:
%
% $$ F_y \approx k_1 \alpha  - k_2\alpha^3 $$
%
% Onde:
%
% $$ k_1 = B C D $$
%
% $$ k_2 = \frac{C D E B^3}{3} + \frac{C D B^3}{3} + \frac{D}{6}(B C)^3 $$
%
%% Modelo de referência
% O modelo de referência usado é o modelo de pneu Pacejka que tem os
% seguintes dados:

pneuPacejkaDados

%% Código e dados
% Código dos dados:

CfaF = c1F*c2F*Fz0*sin(2*atan(FzF/(c2F*Fz0))); % Cfa em função de Fz
CfaR = c1R*c2R*Fz0*sin(2*atan(FzR/(c2R*Fz0))); % Cfa em função de Fz

Cfa0F = c1F*c2F*Fz0*sin(2*atan(Fz0/(c2F*Fz0))); % Cfa para Fz0
Cfa0R = c1R*c2R*Fz0*sin(2*atan(Fz0/(c2R*Fz0))); % Cfa para Fz0

Dy0 = muy0*Fz0;

By0F = Cfa0F/(CyF*Dy0); % Stiffness factor
By0R = Cfa0R/(CyR*Dy0); % Stiffness factor

% Modelo Sadri Equivalente
k1F = CyF*By0F*Dy0;
k2F = (Dy0*EyF*CyF*By0F^3)/3 + (Dy0*CyF*By0F^3)/3 + Dy0/6*(CyF*By0F)^3;
k1R = CyR*By0R*Dy0;
k2R = (Dy0*EyR*CyR*By0R^3)/3 + (Dy0*CyR*By0R^3)/3 + Dy0/6*(CyR*By0R)^3;

pneuDadosFrente = [k1F k2F];
pneuDadosTras = [k1R k2R];

%% Ver também
%
% <index.html Início> | <pneuDoc.html Modelo de pneu>
%