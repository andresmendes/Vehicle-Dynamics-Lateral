function Fy = pneuLinearFun(deriva,pneuDados)

% Parâmetros do pneu
C = pneuDados(1); % Coeficiente de rigidez de curva [N/rad]

% Ângulo de deriva
alpha = deriva; % [rad]

% Força lateral
Fy = -C*alpha;