clear,clc
%% Verificando a equação de movimento
syms deltaf vx vy r dvy dr lf lr Caf Car kf kr m Iz

% Angulos de deriva
alphaf = (vy+lf*r)/vx-deltaf;
alphar = (vy-lr*r)/vx;
 
% Forcas transversais
Fyf = -2*Caf*(alphaf-kf*alphaf^3);
Fyr = -2*Car*(alphar-kr*alphar^3);
 
% Equaçoes de equilibrio
Eq1 = Fyf*cos(deltaf)+Fyr - m*(dvy+vx*r);
Eq2 = lf*Fyf*cos(deltaf)-lr*Fyr - Iz*dr;
 
% Equações de estado
Mov1 = solve(Eq2,dr);
Mov2 = solve(Eq1,dvy);

%% Obtendo as equações de movimento e jacobiano 

% Componentes da matriz jacobiana
J11 = diff(Mov1,r);
J12 = diff(Mov1,vy);
J21 = diff(Mov2,r);
J22 = diff(Mov2,vy);

J = [J11 J12 ; J21 J22];