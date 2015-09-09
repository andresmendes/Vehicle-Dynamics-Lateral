clear,clc
%% Descriçao
% Obtençao das equaçoes de movimento e matriz jacobiana.

%% Angulos de deriva
% Angulos de deriva linearizados

syms ALPHAT dPSI a b v DELTA

% Angulos de deriva
ALPHAF = ALPHAT + a*dPSI/v - DELTA; % Dianteiro
ALPHAR = ALPHAT - b*dPSI/v;         % Traseiro

%% Modelo de pneu
% Modelo nao linear documentado em Pacejka 2006

syms c1F c1R c2F c2R Fz0 FzF FzR CfaF CfaR Cfa0F Cfa0R muy muy0 CyF CyR...
     EyF EyR Dy0F Dy0R

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

%% Equaçoes de equilibrio
% Equaçoes de equilibrio linearizadas
syms m I

Mov1 = (FyF*a - FyR*b)/I;            % ddPSI
Mov2 = (FyF + FyR - m*v*dPSI)/(m*v); % dALPHAT

%% Obtendo as equações de movimento e jacobiano 

% Componentes da matriz jacobiana
J11 = diff(Mov1,dPSI);
J12 = diff(Mov1,ALPHAT);
J21 = diff(Mov2,dPSI);
J22 = diff(Mov2,ALPHAT);

J = [J11 J12;...
     J21 J22];


