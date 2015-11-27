%% Equacionamento
% Este script realiza o desenvolvimento analítico das equações de movimento de um
% modelo bicicleta de veículo articulado com 4 graus de liberdade.
%
%% Variáveis simbólicas
% Definindo as variáveis simbólicas.
%

% Iniciando o código
clear all   % Limpando o workspace
close all   % Fechando as figuras
clc         % Limpando o command window


% Estados e suas derivadas
syms PSI dPSI ddPSI ALPHAT dALPHAT  PHI dPHI ddPHI VEL dVEL
% Dados do veículo
syms mT mS IT IS a b c d e
% Esterçamento
syms DELTA
% Forças
syms FxF FyF FxR FyR FxA FyA FxM FyM

%% Velocidades e acelerações
% Definindo os vetores velocidade e aceleração em diversos pontos.
%

% Matrizes de mudança de base entre a base S e a base T.

S2T = [cos(PHI) sin(PHI) 0;...
      -sin(PHI) cos(PHI) 0;...
         0           0   1];

T2S = simplify(inv(S2T));

% Velocidade no centro de massa T
vT = [VEL*cos(ALPHAT) ; VEL*sin(ALPHAT) ; 0];                   % BASE T

% Aplicando o poisson da velocidade
vF = vT + cross([0 ; 0 ; dPSI],[a ; 0 ; 0]);                    % BASE T
vR = vT + cross([0 ; 0 ; dPSI],[-b ; 0 ; 0]);                   % BASE T
vA = vT + cross([0 ; 0 ; dPSI],[-(b+c) ; 0 ; 0]);               % BASE T

vAnaBaseS = simplify(T2S*vA);                                   % BASE S

vM = vAnaBaseS + cross([0 ; 0 ; dPSI-dPHI],[-(d+e) ; 0 ; 0]);   % BASE S

% Aceleração no centro de massa T
AT = [dVEL*cos(ALPHAT) - VEL*(dALPHAT+dPSI)*sin(ALPHAT) ;...
      dVEL*sin(ALPHAT) + VEL*(dALPHAT+dPSI)*cos(ALPHAT) ;...
                            0                          ];       % BASE T

% Aplicando o poisson da aceleração
AA = AT + cross([0;0;ddPSI],[-(b+c);0;0]) + cross([0;0;dPSI],...
    cross([0;0;dPSI],[-(b+c);0;0]));                            % BASE T

AAnaBaseS = T2S*AA;                                             % BASE S

AS = AAnaBaseS + cross([0;0;ddPSI-ddPHI],[-d;0;0]) + ...
    cross([0;0;dPSI-dPHI],cross([0;0;dPSI-dPHI],[-d;0;0]));     % BASE S

%% Ângulo de deriva
% Definindo os ângulos de deriva nos eixos.
%

ALPHAF = atan(vF(2)/vF(1)) - DELTA;
ALPHAR = atan(vR(2)/vR(1));
ALPHAA = atan(vA(2)/vA(1));
ALPHAM = atan(vM(2)/vM(1));

%% Forças e Momentos
% Definindo os vetores força e momento.
%

% Força no eixo dianteiro F
FF = [FxF*cos(DELTA) - FyF*sin(DELTA) ;...
      FxF*sin(DELTA) + FyF*cos(DELTA) ;...
                     0               ];                         % BASE T
% Força no eixo traseiro R
FR = [FxR ; FyR ; 0];                                           % BASE T
% Força na articulação S sobre o semirreboque
FAsemi = -[FxA ; FyA ; 0];                                      % BASE S (Na direção negativa da base)
% Força na articulação S sobre o caminhão-trator
FAtrator = -S2T*FAsemi;                                         % BASE T
% Força no eixo do semirreboque
FM = [FxM ; FyM ; 0];                                           % BASE S

% Braços de alavanca
% Caminhão-trator
FTvet = [a ; 0 ; 0];                % Vetor do ponto T ao F - BASE T
RTvet = [- b ; 0 ; 0];              % Vetor do ponto T ao R - BASE T
ATvet = [- (b+c) ; 0 ; 0];          % Vetor do ponto T ao A - BASE T
% Semirreboque
ASvet = [d ; 0 ; 0];                % Vetor do ponto S ao A - BASE S
MSvet = [-e ; 0 ; 0];               % Vetor do ponto S ao M - BASE S
% Momentos
% Caminhão-trator
MomF = cross(FTvet,FF);             % Gerado pela força FF em relação a T
MomR = cross(RTvet,FR);             % Gerado pela força FR em relação a T
MomAtrator = cross(ATvet,FAtrator); % Gerado pela força FSC em relação a T
% Semirreboque
MomAsemi = cross(ASvet,FAsemi);     % Gerado pela força FAsemi em relação a S
MomM = cross(MSvet,FM);             % Gerado pela força FM em relação a S

%% TMB e TMA
% Equacionando o teorema do movimento do baricentro (TMB) e o teorema da
% quantidade de movimento angular (TMA).

% Caminhão-trator
TMBtrator = FF + FR + FAtrator - mT*AT;
TMAtrator = MomF + MomR + MomAtrator - IT*ddPSI;
% Semirreboque
TMBsemi = FAsemi + FM - mS*AS;
TMAsemi = MomAsemi + MomM - IS*(ddPSI-ddPHI);

% Eliminando o vínculo
% As equações do TMB do semirreboque apresentam as forças de vínculo FxA e
% FyA separadamente. Logo, elas serão usadas para eliminar as forças de
% vínculo nas equações restantes
FxAeq = solve(TMBsemi(1),FxA);
FyAeq = solve(TMBsemi(2),FyA);

%% Equação de estado
% A equação de estado é desenvolvida na forma:
%
% $$ M \dot{x} = f(x) $$
%
% Onde $M$ é a matriz de massa, $x$ é o vetor de estados e $f$ é uma função
% vetorial não linear.

% Definindo o vetor de estados.
dX = [ddPSI ; dALPHAT ; ddPHI ; dVEL ; dPHI];

% As equações eq1, eq2, eq3 e eq4 (a seguir) descrevem as relações de equilíbrio
% sem as forças de vínculo. Logo, as equações usadas são o TMBtrator e os dois
% TMA.
eq1 = subs(TMBtrator(1),[FxA FyA],[FxAeq FyAeq]);
eq2 = subs(TMBtrator(2),[FxA FyA],[FxAeq FyAeq]);
eq3 = subs(TMAtrator(3),[FxA FyA],[FxAeq FyAeq]);
eq4 = subs(TMAsemi(3),[FxA FyA],[FxAeq FyAeq]);
% Para a integração do sistema é necessário acrescentar a equação trivial eq5
eq5 = dPHI;

% Coeficientes da matriz de massa
[M11,t11] = coeffs(eq1,ddPSI);
[M12,t12] = coeffs(M11(2),dALPHAT);
[M13,t13] = coeffs(M12(2),ddPHI);
[M14,t14] = coeffs(M13(2),dVEL);

[M21,t21] = coeffs(eq2,ddPSI);
[M22,t22] = coeffs(M21(2),dALPHAT);
[M23,t23] = coeffs(M22(2),ddPHI);
[M24,t24] = coeffs(M23(2),dVEL);

[M31,t31] = coeffs(eq3,ddPSI);
[M32,t32] = coeffs(M31(2),dALPHAT);
[M33,t33] = coeffs(M32(2),ddPHI);
[M34,t34] = coeffs(M33(2),dVEL);

[M41,t41] = coeffs(eq4,ddPSI);
[M42,t42] = coeffs(M41(2),dALPHAT);
[M43,t43] = coeffs(M42(2),ddPHI);
[M44,t44] = coeffs(M43(2),dVEL);

% Matriz de massa do sistema
% Com a equação trivial decorrente da segunda derivada
M = [M11(1) M12(1) M13(1) M14(1) 0;...
     M21(1) M22(1) M23(1) M24(1) 0;...
     M31(1) M32(1) M33(1) M34(1) 0;...
     M41(1) M42(1) M43(1) M44(1) 0;...
       0      0      0      0    1];

M = -simplify(M);   % É negativo se tratando da equação acima

% Função vetorial
% Composta pelo resto da última verificação de coeficiente.
f = [M14(2);...
     M24(2);...
     M34(2);...
     M44(2);...
     dPHI];

f = simplify(f);

%% Equacionamento explícito
% Equação matricial desenvolvida deve ser igual à solução do sistemas de equações
% (eq1, eq2, eq3, eq4 e eq5) resolvido para os estados contidos no vetor de
% estados.
%

% Formulação com matriz de massa
EQmatriz = M\f;

% As equações de movimento são obtidas resolvendo o sistema de equações
% diferenciais para o termo de derivada com maior ordem de cada variável.
[sol1, sol2, sol3, sol4] = solve(eq1==0,eq2==0,eq3==0,eq4==0,ddPSI,dALPHAT,ddPHI,dVEL);

% Verificação
% Subtraindo as equações equivalentes obtidas através do equacionamento matricial
% e resolvendo o sistema de equações.
verifM1 = simplify(EQmatriz(1) - sol1);
verifM2 = simplify(EQmatriz(2) - sol2);
verifM3 = simplify(EQmatriz(3) - sol3);
verifM4 = simplify(EQmatriz(4) - sol4);

%% Ver também
%
% <index.html Início>
%
