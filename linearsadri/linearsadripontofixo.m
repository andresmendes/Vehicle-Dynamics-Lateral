%% Descrição
% Cálculo dos pontos fixos do sistema

% EM CONSTRUÇÃO EM CONSTRUÇÃO EM CONSTRUÇÃO EM CONSTRUÇÃO EM CONSTRUÇÃO

%% Parametros do veiculo
Caf0 = 57300; % [N/rad]
Car0 = 57300; % [N/rad]
kf0 = 4.87;
kr0 = 4.87;
Iz0 = 6550; % [kgm2]
m0 = 2527; % [kg]
lf0 = 1.37; % [m]
lr0 = 1.86; % [m]
vx0 = 20; % [m/s]
deltaf0 = 0*pi/180; % [rad]

%% Cálculo dos pontos fixos
drEQ = subs(Mov2,[vx deltaf Caf Car kf kr lf lr m Iz],[vx0 deltaf0 Caf0 Car0 kf0 kr0 lf0 lr0 m0 Iz0])
dvyEQ = subs(Mov1,[vx deltaf Caf Car kf kr lf lr m Iz],[vx0 deltaf0 Caf0 Car0 kf0 kr0 lf0 lr0 m0 Iz0])

[v r] = vpasolve([dvyEQ==0, drEQ==0],[vy, r])