function [ dx ] = naolinearandrelyapunovext(t,x,VEICULODADOS,PNEUDADOS)
% Dadosdo veículo
m = VEICULODADOS(1);   % massa do veiculo [kg]
I = VEICULODADOS(2);   % momento de inercia [kg]
a = VEICULODADOS(3);   % distancia do eixo dianteiro ao centro de massa [m]
b = VEICULODADOS(4);   % distancia do eixo dianteiro ao centro de massa [m]
v = VEICULODADOS(5);     % módulo da velocidade do centro de massa [m/s]
DELTA = VEICULODADOS(6); % Esterçamento

g = 9.81;   % Aceleração da gravidade [m/s2]

% Estados
dPSI = x(1);
ALPHAT = x(2);
w11 = x(3);
w21 = x(4);
w12 = x(5);
w22 = x(6);

W = [w11 w12;...
     w21 w22];

% Conversao do angulo para que fique so entre -180 e 180
% met = sin(ALPHAT); % verifica se o angulo esta no plano esquerdo ou direito do carro FOLHA 15
% if met > 0
%     ANGULO = ALPHAT - floor(ALPHAT/(pi))*(pi);
% else
%     ANGULO = ALPHAT - floor(ALPHAT/(pi))*(pi) - pi;
% end
% ALPHAT = ANGULO; 
 
% Angulos de deriva não linear
ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA; % Dianteiro
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));         % Traseiro


% ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
% ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));
% 
%     
%     if (v*cos(ALPHAT))<=0
%         ALPHAF = -atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
%         ALPHAR = -atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));
%     end

% Força lateral do modelo de Pacejka
Fz0 = PNEUDADOS(1);
muy0 = PNEUDADOS(2);

% Parâmetros do pneu frente
CyF = PNEUDADOS(3);
EyF = PNEUDADOS(4);
c1F = PNEUDADOS(5);
c2F = PNEUDADOS(6);

% Parâmetros do pneu tras
CyR = PNEUDADOS(7);
EyR = PNEUDADOS(8);
c1R = PNEUDADOS(9);
c2R = PNEUDADOS(10);

% Condições de operação
muy = PNEUDADOS(11);
FzF = PNEUDADOS(12); 
FzR = PNEUDADOS(13);


CfaF = c1F*c2F*Fz0*sin(2*atan(FzF/(c2F*Fz0))); % Cfa em função de Fz
CfaR = c1R*c2R*Fz0*sin(2*atan(FzR/(c2R*Fz0))); % Cfa em função de Fz

Cfa0F = c1F*c2F*Fz0*sin(2*atan(Fz0/(c2F*Fz0))); % Cfa para Fz0
Cfa0R = c1R*c2R*Fz0*sin(2*atan(Fz0/(c2R*Fz0))); % Cfa para Fz0

alphaeqF = CfaF/Cfa0F*muy0/muy*Fz0/FzF*ALPHAF; % alpha equivalente
alphaeqR = CfaR/Cfa0R*muy0/muy*Fz0/FzR*ALPHAR; % alpha equivalente

Dy0F = muy0*Fz0;
Dy0R = muy0*Fz0;

By0F = Cfa0F/(CyF*Dy0F); % Stiffness factor
By0R = Cfa0R/(CyR*Dy0R); % Stiffness factor

Fy0F = Dy0F*sin(CyF*atan(By0F*alphaeqF-EyF*(By0F*alphaeqF-atan(By0F*alphaeqF))));
Fy0R = Dy0R*sin(CyR*atan(By0R*alphaeqR-EyR*(By0R*alphaeqR-atan(By0R*alphaeqR))));

FyF = -muy/muy0*FzF/Fz0*Fy0F;
FyR = -muy/muy0*FzR/Fz0*Fy0R;

% Equações de estado
dx(1,1) = (FyF*cos(DELTA)*a - FyR*b)/I; % ddPSI
dx(2,1) = (FyF*cos(DELTA) + FyR - m*v*cos(ALPHAT)*dPSI)/(m*v*cos(ALPHAT)); % dALPHAT

% Matriz jacobiana
J11 = ((CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT)))^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT))))^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy))^2 + 1))/I;
J12 = ((CyR*FzR*b*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT)))^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*a*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT))))^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy))^2 + 1))/I;
J21 = -(m*v*cos(ALPHAT) + (CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*(EyR*((Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT)))^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*b*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R))))/(CyR*FzR*muy*v*cos(ALPHAT)*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))^2 + 1) - (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*(EyF*((Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT))))^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*a*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F))))/(CyF*FzF*muy*v*cos(ALPHAT)*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy))^2 + 1))/(m*v*cos(ALPHAT));
J22 = (dPSI*m*v*sin(ALPHAT) - (CyR*FzR*muy*cos(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*(EyR*((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1R^2*c2R^2*sin(2*atan(FzR/(Fz0*c2R)))^2*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT)))^2)/(CyR^2*FzR^2*muy^2) + 1))) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*((sin(ALPHAT)*(b*dPSI - v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) - 1))/(CyR*FzR*muy*((b*dPSI - v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))^2 + 1) + (CyF*FzF*muy*cos(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*(EyF*((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1)*((Fz0^2*c1F^2*c2F^2*sin(2*atan(FzF/(Fz0*c2F)))^2*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT))))^2)/(CyF^2*FzF^2*muy^2) + 1))) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*((sin(ALPHAT)*(a*dPSI + v*sin(ALPHAT)))/(v*cos(ALPHAT)^2) + 1))/(CyF*FzF*muy*((a*dPSI + v*sin(ALPHAT))^2/(v^2*cos(ALPHAT)^2) + 1))))/((EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy))^2 + 1))/(m*v*cos(ALPHAT)) + (sin(ALPHAT)*(FzR*muy*sin(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))) + FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA) - dPSI*m*v*cos(ALPHAT)))/(m*v*cos(ALPHAT)^2);
      
J =[J11 J12;...
    J21 J22];

dW = J*W;

dx(3) = dW(1,1);
dx(4) = dW(2,1);
dx(5) = dW(1,2);
dx(6) = dW(2,2);

end