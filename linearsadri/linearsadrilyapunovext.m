function y = linearsadrilyapunovext (t,x,VEICULODADOS,PNEUDADOS)

% Estados
r = x(1);
vy  = x(2);

% Matriz da equação variacional
W11 = x(3);
W21 = x(4);
W12 = x(5);
W22 = x(6);

W = [W11 W12;...
     W21 W22];

% Dadosdo veículo
m = VEICULODADOS(1);   % massa do veiculo [kg]
Iz = VEICULODADOS(2);   % momento de inercia [kg]
lf = VEICULODADOS(3);   % distancia do eixo dianteiro ao centro de massa [m]
lr = VEICULODADOS(4);   % distancia do eixo dianteiro ao centro de massa [m]
vx = VEICULODADOS(5);     % módulo da velocidade do centro de massa [m/s]
deltaf = VEICULODADOS(6); % Esterçamento

% Dados
Caf = PNEUDADOS(1); % [N/rad]
Car = PNEUDADOS(2); % [N/rad]
kf = PNEUDADOS(3);
kr = PNEUDADOS(4);

% Equação do sistema
dr  = (2*Car*lr*((vy - lr*r)/vx - (kr*(vy - lr*r)^3)/vx^3) - 2*Caf*lf*cos(deltaf)*(kf*(deltaf - (vy + lf*r)/vx)^3 - deltaf + (vy + lf*r)/vx))/Iz;
dvy = -(2*Car*((vy - lr*r)/vx - (kr*(vy - lr*r)^3)/vx^3) + 2*Caf*cos(deltaf)*(kf*(deltaf - (vy + lf*r)/vx)^3 - deltaf + (vy + lf*r)/vx) + m*r*vx)/m;

% Equação variacional
J11 = -(2*Car*lr*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*lf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J12 = (2*Car*lr*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) - 2*Caf*lf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/Iz;
J21 = -(m*vx - 2*Car*(lr/vx - (3*kr*lr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(lf/vx - (3*kf*lf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;
J22 = -(2*Car*(1/vx - (3*kr*(vy - lr*r)^2)/vx^3) + 2*Caf*cos(deltaf)*(1/vx - (3*kf*(deltaf - (vy + lf*r)/vx)^2)/vx))/m;

J = [J11 J12;...
     J21 J22];

dW = J*W;

% Saída
y(1,1) = dr;
y(2,1) = dvy;
y(3,1) = dW(1,1);
y(4,1) = dW(2,1);
y(5,1) = dW(1,2);
y(6,1) = dW(2,2);

end
