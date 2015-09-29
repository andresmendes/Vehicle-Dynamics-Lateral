clear all
clc

syms B C D E alpha

x = B*alpha;
%y = B*alpha - E*B*alpha -E*(x - 1/3*x^3);
y = B*alpha - E*B*alpha + E*(x - 1/3*x^3);
%y =  -E*(x-1/3*x^3);
z = C*(y - 1/3*y^3);

f = D*(z - 1/6*z^3);

[c,t] = coeffs(f,alpha)

%% Resultados
% C*D*(B - 2*B*E) * ALPHA
% D*(C*((B^3*E)/3 - (B - 2*B*E)^3/3) - (C^3*(B - 2*B*E)^3)/6) * ALPHA^3
% Logo,
% k1 = C*D*(B - 2*B*E);
% k2 = D*(C*((B^3*E)/3 - (B - 2*B*E)^3/3) - (C^3*(B - 2*B*E)^3)/6);