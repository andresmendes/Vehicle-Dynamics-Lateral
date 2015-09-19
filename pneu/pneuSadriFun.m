function Fy = pneuSadriFun(deriva,pneuDados)

% Parâmetros do pneu
Ca = pneuDados(1);
k = pneuDados(2);

% Ângulo de deriva
alpha = deriva; % [rad]

% Força lateral
Fy = -2*Ca*(alpha-k*alpha.^3);