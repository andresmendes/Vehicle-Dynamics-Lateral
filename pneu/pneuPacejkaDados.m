% Dados do pneu PACEJKA
% Os parâmetros dos eixos dianteiros e traseiros visam ser equivalentes ao
% modelo simplificado usado por SADRI E WU 2013. Isso é feito da seguinte
% utilizando as seguintes equivalencias:
% * Mesmo coeficiente de rigidez de curva para angulos de deriva pequenos
% * Mesma força lateral máxima
% Dados retirados do script de pneus (pneusadriXpacejka.m) comparando os
% dois modelos.
    % muy0 = 0.8;
    % Fz0 = 2.4985e+04;
    % muy = muy0;
    % FzF = Fz0;
    % Cy = 1.5;
    % Ey = -2;
    % c1 = 3.5899;
    % c2 = 1.33;
% Condições nominais
Fz0 = 2.4985e+04; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal
% Condições de operação
FzF = Fz0; % Eixo Dianteiro
FzR = Fz0; % Eixo Traseiro
muy = muy0; % Coeficiente de atrito de operação
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

pneuDadosFrente = [Fz0 muy0 CyF EyF c1F c2F FzF muy];
pneuDadosTras = [Fz0 muy0 CyR EyR c1R c2R FzR muy];