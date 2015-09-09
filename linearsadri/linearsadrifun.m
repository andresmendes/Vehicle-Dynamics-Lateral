function y = linearsadrifun (t,x,VEICULODADOS,PNEUDADOS)

% Dadosdo veículo
m = VEICULODADOS(1);   % massa do veiculo [kg]
Iz = VEICULODADOS(2);   % momento de inercia [kg]
lf = VEICULODADOS(3);   % distancia do eixo dianteiro ao centro de massa [m]
lr = VEICULODADOS(4);   % distancia do eixo dianteiro ao centro de massa [m]
v = VEICULODADOS(5);     % módulo da velocidade do centro de massa [m/s]
deltaf = VEICULODADOS(6); % Esterçamento

% Dados
Caf = PNEUDADOS(1); % [N/rad]
Car = PNEUDADOS(2); % [N/rad]
kf = PNEUDADOS(3);
kr = PNEUDADOS(4);

% Estados
r = x(1);
vy  = x(2);
PSI = x(3);
X = x(4);
Y = x(5);

% Equações do artigo que parecem estar erradas
% dvy = (-2*Caf*cos(deltaf)*((vy+lf*r)/vx-deltaf)*(1+kf*((vy+lf*r)/vx-deltaf)^2))/m...
%         - (2*Car*((vy-lr*r)/vx)*(1+kr*((vy-lr*r)/vx)^2))/m - vx*r;
% dr =  (-2*Caf*lf*cos(deltaf)*((vy+lf*r)/vx-deltaf)*(1+kf*((vy+lf*r)/vx-deltaf)^2))/Iz...
%         + (2*Car*lr*((vy-lr*r)/vx)*(1+kr*((vy-lr*r)/vx)^2))/Iz;

% Equações verificadas por mim que parecem estar certas
dr  = (2*Car*lr*((vy - lr*r)/v - (kr*(vy - lr*r)^3)/v^3) - 2*Caf*lf*cos(deltaf)*(kf*(deltaf - (vy + lf*r)/v)^3 - deltaf + (vy + lf*r)/v))/Iz;
dvy = -(2*Car*((vy - lr*r)/v - (kr*(vy - lr*r)^3)/v^3) + 2*Caf*cos(deltaf)*(kf*(deltaf - (vy + lf*r)/v)^3 - deltaf + (vy + lf*r)/v) + m*r*v)/m;

% Saida dos estados
y(1,1) = dr;
y(2,1) = dvy;

%%%%%%%%%%%%%%% PARA ANIMACAO %%%%%%%%%%%%%%%
% Obtenção da orientação
y(3,1) = r; % dPSI

% Equações adicionais para o posicionamento
    % Não necessárias para a dinâmica em guinada
    
ALPHAT = asin(vy/v);
y(4,1) = v*cos(ALPHAT + PSI); % X
y(5,1) = v*sin(ALPHAT + PSI); % Y

end