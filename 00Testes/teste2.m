

% Condições nominais
Fz0 = 2.4985e+04; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal
% Condições de operação
Fz = Fz0; % Eixo Dianteiro
muy = muy0; % Coeficiente de atrito de operação
% Coeficientes experimentais do pneu dianteiro
C = 1.5;
E = -2;
c1 = 3.5899;
c2 = 1.33;

Cfa = c1.*c2.*Fz0.*sin(2.*atan(Fz./(c2.*Fz0))); % Cfa em função de Fz

Cfa0 = c1.*c2.*Fz0.*sin(2.*atan(Fz0./(c2.*Fz0))); % Cfa para Fz0

D = muy0.*Fz0;

B = Cfa0./(C.*D); % Stiffness factor


alpha = (0:0.001:1).*pi./180; % Ângulo de deriva [rad]
Fy = -D.*((C.^3.*(B.*alpha - E.*(B.*alpha - (B.^3.*alpha.^3)./3) + (E.*(B.*alpha - (B.^3.*alpha.^3)./3) - B.*alpha + B.*E.*alpha).^3./3 - B.*E.*alpha).^3)./6 - C.*(B.*alpha - E.*(B.*alpha - (B.^3.*alpha.^3)./3) + (E.*(B.*alpha - (B.^3.*alpha.^3)./3) - B.*alpha + B.*E.*alpha).^3./3 - B.*E.*alpha))
