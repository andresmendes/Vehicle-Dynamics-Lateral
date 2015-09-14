clear,clc
%% teste muito louco

%% Dados do veiculo
% Mesmos dados apresentados por SADRI E WU 2013
m = 2527;   % massa do veiculo [kg]
I = 6550;   % momento de inercia [kg]
b = 1.37;   % distancia do eixo dianteiro ao centro de massa [m]
a = 1.86;   % distancia do eixo dianteiro ao centro de massa [m]
v = 20;     % módulo da velocidade do centro de massa [m/s]

DELTA = 0*pi/180; % esterçamento do eixo dianteiro [grau]

%% Dados do pneu
Fz0 = 2.4985e+04; % Carga vertical nominal
muy0 = 0.8; % Coeficiente de atrito nominal
% Coeficientes experimentais do pneu dianteiro
CyF = 1.5;
EyF = -2;
c1F = 3.5899;
c2F = 1.33;
% Coeficientes experimentaisdo pneu traseiro
CyR = 1.5;
EyR = -2;
c1R = 3.5899;
c2R = 1.33;
% Condições de operação
muy = muy0;
FzF = Fz0; 
FzR = Fz0;


%% Estados
num = 100; % numero da resulução20
dPSImax = 20;
vymax = 20; 

res = 0.5; % resolução

dPSIvet = [-dPSImax:res:dPSImax];
ALPHATvet = asin([-vymax:res:vymax]/v);


for i = 1:length(dPSIvet)
    for j = 1:length(ALPHATvet)
 
dPSI = dPSIvet(i);
ALPHAT = ALPHATvet(j);

%% Ângulos de deriva
ALPHAF = ALPHAT + a*dPSI/v - DELTA; % Dianteiro
ALPHAR = ALPHAT - b*dPSI/v;         % Traseiro

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

%% Equações de estado
ddPSI(i,j) = (FyF*a - FyR*b)/I;           % ddPSI
dALPHAT(i,j) = (FyF + FyR - m*v*dPSI)/(m*v); % dALPHAT
    end
end

z = zeros(length(dPSIvet),length(ALPHATvet));

for j = 1:length(ALPHATvet)-1
    z(1,j+1) = -dALPHAT(1,j)*(ALPHATvet(j+1)-ALPHATvet(j)) + z(1,j);
end

for j = 1:length(ALPHATvet)
    for i = 1:length(dPSIvet)-1
        z(i+1,j) = -ddPSI(i,j)*(dPSIvet(i+1)-dPSIvet(i)) + z(i,j);
    end
end

% Acertando a origem como saindo do zero
z = z - z(dPSImax/res+1,vymax/res+1);

%[X,Y] = meshgrid(dPSIvet,ALPHATvet);

figure(1)
surface(ALPHATvet,dPSIvet,z)

figure(2)
[c,h]=contour(ALPHATvet,dPSIvet,z,10);
set(h,'ShowText','on')