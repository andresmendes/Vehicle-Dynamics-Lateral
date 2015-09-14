clear,clc

%% Angulo de deriva

% Definindo variaveis simbolicas
syms v ALPHAT dPSI a b DELTA

ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));

%% Aceleracao

% Definindo variaveis simbolicas
syms dv dALPHAT

at = [dv*cos(ALPHAT) - v*sin(ALPHAT)*(dALPHAT + dPSI);...
      dv*sin(ALPHAT) + v*cos(ALPHAT)*(dALPHAT + dPSI)];

% %% Modelo de Pneu
% 
syms Fz0 muy0 

syms CyF EyF c1F c2F
syms CyR EyR c1R c2R

syms muy FzF FzR

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


%% Forcas

% Definindo variaveis simbolicas
syms FxF FxR

FF = [FxF*cos(DELTA) + FyF*(-sin(DELTA));...
      FxF*sin(DELTA) + FyF*cos(DELTA)];
  
FR = [FxR ; FyR];
  
%% Distancias

FT = [a ; 0]; % Vetor posicao de F em relação a T
RT = [-b ; 0]; % Vetor posicao de R em relação a T

%% TMB

syms m

TMB = FF + FR - m*at;

%% TMA

syms I ddPSI

TMA = FT(1)*FF(2) + RT(1)*FR(2) - I*ddPSI;

%% Equações de estado

eqdv = solve(TMB(1),dv);
eqdALPHAT = solve(TMB(2),dALPHAT);

eq1 = subs(TMB(2),dv,eqdv);
eq2 = subs(TMB(1),dALPHAT,eqdALPHAT);

EQ1 = solve(TMA,ddPSI)
EQ2 = solve(eq1,dALPHAT)
EQ3 = solve(eq2,dv)

% Forca longitudinal necessária para manter a velocidade constante
%F = solve(EQ3,FxF);

EQaux = EQ3*cos(ALPHAT) - v*sin(ALPHAT)*(dALPHAT + dPSI);

F = solve(EQaux,FxF);



%% Montando a matriz jacobiana

J11 = diff(EQ1,dPSI);
J21 = diff(EQ2,dPSI);
J31 = diff(EQ3,dPSI);
J12 = diff(EQ1,ALPHAT);
J22 = diff(EQ2,ALPHAT);
J32 = diff(EQ3,ALPHAT);
J13 = diff(EQ1,v);
J23 = diff(EQ2,v);
J33 = diff(EQ3,v);

J = [J11 J12 J13;...
     J21 J22 J23;...
     J31 J32 J33];
 
 
