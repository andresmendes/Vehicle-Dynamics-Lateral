% Coeficientes do modelo de SADRI
Caf = 57300; % [N/rad]
Car = 57300; % [N/rad]

% Coeficientes utilizados no modelo linear
Cf = 2*Caf; % Coeficiente de rigidez de curva [N/rad] 
Cr = 2*Car; % Coeficiente de rigidez de curva [N/rad] 

pneuDadosFrente = [Cf];
pneuDadosTras = [Cr];
