clear all,clc
%% Descrição
% Este script tem como objetivo desenvolver as equações de movimento de um modelo de veículo não linear com 3gdl.

%% Dados do veículo
veiculoDadosScript % Rodando o script de dados do veículo

%% Dados do pneu
cd .. % Saindo da pasta veiculo
cd pneu % Entrando na pasta pneu
pneuPacejkaDados % Rodando os dados do pneu
cd .. % Saindo da pasta pneu
cd veiculo % Voltando para pasta veiculo

%% Definindo variaveis simbolicas
syms ALPHAT dPSI

%% Ângulos de deriva
ALPHAF = atan((v*sin(ALPHAT) + a*dPSI)/(v*cos(ALPHAT))) - DELTA;
ALPHAR = atan((v*sin(ALPHAT) - b*dPSI)/(v*cos(ALPHAT)));

%% Modelo de pneu
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

%% Forças longitudinais
FxF = 0;
FxR = 0;

%% Equações de movimento
ddPSI = (FyF*a*cos(DELTA) - FyR*b + FxF*a*sin(DELTA))/I;
dALPHAT = (FyR + FyF*cos(DELTA) + FxF*sin(DELTA) - m*(dPSI*v*cos(ALPHAT) + (sin(ALPHAT)*(FxR + 	FxF*cos(DELTA) - ...
	FyF*sin(DELTA) + dPSI*m*v*sin(ALPHAT)))/(m*cos(ALPHAT))))/(m*(v*cos(ALPHAT) + (v*sin(ALPHAT)^2)/cos(ALPHAT)));
dV = (FxR*cos(ALPHAT) + FyR*sin(ALPHAT) - FyF*cos(ALPHAT)*sin(DELTA) + FyF*cos(DELTA)*sin(ALPHAT) + ...
	FxF*sin(ALPHAT)*sin(DELTA) + FxF*cos(ALPHAT)*cos(DELTA))/(m*cos(ALPHAT)^2 + m*sin(ALPHAT)^2);

%% Ponto fixo
%[dPSI ALPHAT v] = vpasolve([ddPSI==0, dALPHAT==0],[dV, v])
[dPSIresultado ALPHATresultado] = vpasolve([ddPSI==0, dALPHAT==0],[dPSI, ALPHAT])

%% Verificação
valor = [dPSIresultado ALPHATresultado];

ddPSIvalor = subs(ddPSI,[dPSI ALPHAT],valor)
dALPHATvalor = subs(dALPHAT,[dPSI ALPHAT],valor)





% %% Montando a matriz jacobiana

% J11 = diff(EQ1,dPSI);
% J21 = diff(EQ2,dPSI);
% J31 = diff(EQ3,dPSI);
% J12 = diff(EQ1,ALPHAT);
% J22 = diff(EQ2,ALPHAT);
% J32 = diff(EQ3,ALPHAT);
% J13 = diff(EQ1,v);
% J23 = diff(EQ2,v);
% J33 = diff(EQ3,v);

% J = [J11 J12 J13;...
%      J21 J22 J23;...
%      J31 J32 J33];
 
 
