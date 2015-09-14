function [ dx ] = nonlinandrefun(t,x,DADOS)

% Parâmetros do veículo
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inercia [kg]
a = DADOS(1);   % distancia do eixo dianteiro ao centro de massa [m]
b = DADOS(2);   % distancia do eixo dianteiro ao centro de massa [m]
g = 9.81;   % Aceleração da gravidade [m/s2]
%v = 20;     % módulo da velocidade do centro de massa [m/s]


DELTA = DADOS(3);

%DELTA = 10*pi/180; % Esterçamento

% Estados
dPSI = x(1);
ALPHAT = x(2);
v = x(3);
PSI = x(4);

ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));

  
    
%     if (v*cos(ALPHAT))<=0%ALPHAT(i)>=pi/2 & ALPHAT(i)<3/2*pi
%         if (v*sin(ALPHAT) + a*dPSI)>0
%             ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA + pi;
%         else
%             ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA - pi;
%         end
%     
%         if (v*sin(ALPHAT) - b*dPSI)>0
%             ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT))) + pi;
%         else
%             ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT))) - pi;
%         end
%     end
    
    if (v*cos(ALPHAT))<=0
        ALPHAF = -atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
        ALPHAR = -atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));
    end


% % Ângulos de deriva






% Força lateral do modelo de Pacejka
%Fz0 = 3000; % Carga vertical inicial
Fz0 = 15000;
muy0 = 0.8;

% Parâmetros do pneu frente
CyF = 0.9;
EyF = -1;
%c1 = 8; % Original
%c1F = 3.8;
c1F = 8;
c2F = 1.33;

% Parâmetros do pneu tras
CyR = 0.9;
EyR = -1;
%c1 = 8; % Original
c1R = 8;
c2R = 1.33;


% Condições de operação
muy = 0.8;
% FzF = 3000;
% FzR = 3000;
FzF = m*b/(a+b)*g;
FzR = m*a/(a+b)*g;


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


%FyF = -100000*ALPHAF;
%FyR = -100000*ALPHAR;


% Forcas longitudinais
FxF = 0;
FxR = 0;
    % Forca longitudinal necessária para manter a velocidade constante

    
%FxF = -(FxR*cos(ALPHAT) + FzR*muy*sin(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*sin(ALPHAT) - FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(ALPHAT)*sin(DELTA) + FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*sin(ALPHAT))/(cos(ALPHAT)*cos(DELTA) + sin(ALPHAT)*sin(DELTA));
% if FxF>muy*FzF
%     FxF = muy*FzF;
% end
%FxR = -(FyR*sin(ALPHAT) - FyF*cos(ALPHAT)*sin(DELTA) + FyF*cos(DELTA)*sin(ALPHAT) + FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA))/cos(ALPHAT);
    % Forca longitudinal para manter a velocidade LONGITUDINAL constante
% ?

% Equações de estado
% dx(1,1) = (FxF*a*sin(DELTA) - FzR*b*muy*sin(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))) + FzF*a*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA))/I;
% dx(2,1) = (FxF*sin(DELTA) - m*(dPSI*v*cos(ALPHAT) + (sin(ALPHAT)*(FxR + FxF*cos(DELTA) - FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*sin(DELTA) + dPSI*m*v*sin(ALPHAT)))/(m*cos(ALPHAT))) + FzR*muy*sin(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy))) + FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA))/(m*(v*cos(ALPHAT) + (v*sin(ALPHAT)^2)/cos(ALPHAT)));
% dx(3,1) = (FxR*cos(ALPHAT) + FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA) + FzR*muy*sin(CyR*atan(EyR*(atan((Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) - (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)) + (Fz0*c1R*c2R*sin(2*atan(FzR/(Fz0*c2R)))*atan((b*dPSI - v*sin(ALPHAT))/(v*cos(ALPHAT))))/(CyR*FzR*muy)))*sin(ALPHAT) - FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(ALPHAT)*sin(DELTA) + FzF*muy*sin(CyF*atan(EyF*(atan((Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) - (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)) + (Fz0*c1F*c2F*sin(2*atan(FzF/(Fz0*c2F)))*(DELTA - atan((a*dPSI + v*sin(ALPHAT))/(v*cos(ALPHAT)))))/(CyF*FzF*muy)))*cos(DELTA)*sin(ALPHAT))/(m*cos(ALPHAT)^2 + m*sin(ALPHAT)^2);

dx(1,1) = (FyF*a*cos(DELTA) - FyR*b + FxF*a*sin(DELTA))/I;
dx(2,1) = (FyR + FyF*cos(DELTA) + FxF*sin(DELTA) - m*(dPSI*v*cos(ALPHAT) + (sin(ALPHAT)*(FxR + FxF*cos(DELTA) - FyF*sin(DELTA) + dPSI*m*v*sin(ALPHAT)))/(m*cos(ALPHAT))))/(m*(v*cos(ALPHAT) + (v*sin(ALPHAT)^2)/cos(ALPHAT)));
dx(3,1) = (FxR*cos(ALPHAT) + FyR*sin(ALPHAT) - FyF*cos(ALPHAT)*sin(DELTA) + FyF*cos(DELTA)*sin(ALPHAT) + FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA))/(m*cos(ALPHAT)^2 + m*sin(ALPHAT)^2);


% Obtenção da orientação
dx(4,1) = dPSI; % dPSI

% Equações adicionais para o posicionamento
    % Não necessárias para a dinâmica em guinada
    
dx(5,1) = v*cos(ALPHAT + PSI); % X
dx(6,1) = v*sin(ALPHAT + PSI); % Y

end