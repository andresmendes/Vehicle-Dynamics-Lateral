%% Animação
% Fazer a animação do veículo andando após a integração. Este script faz
% uso do <vetor.html vetor>.

%% Sintaxe
% |animacao(XOUT,TOUT,ALPHAFRONT,ALPHAREAR,VELF,VELR,VELT,DADOS)|
%
%% Argumentos
% Lista de entradas da função:
%
% <html> <table border=1 width="97%"> 
% <tr> <td width="30%"><tt>XOUT</tt></td> <td width="70%">Estados de saída do integrador.</td> </tr>
% <tr> <td><tt>TOUT</tt></td> <td>Tempo de saída do integrador.</td> </tr>
% <tr> <td><tt>ALPHAFRONT</tt></td> <td>Ângulo de deriva no eixo dianteiro.</td> </tr>
% <tr> <td><tt>ALPHAREAR</tt></td> <td>Ângulo de deriva no eixo traseiro.</td> </tr>
% <tr> <td><tt>VELF</tt></td> <td>Módulo da velocidade no eixo dianteiro.</td> </tr>
% <tr> <td><tt>VELR</tt></td> <td>Módulo da velocidade no eixo traseiro.</td> </tr>
% <tr> <td><tt>VELT</tt></td> <td>Módulo da velocidade no centro de massa.</td> </tr>
% <tr> <td><tt>DADOS</tt></td> <td>Dados do veículo.</td> </tr>
% </table> </html>
% 
%% Definindo as variáveis locais
%

% Iniciando a função
function animacao(XOUT,TOUT,ALPHAFRONT,ALPHAREAR,VELF,VELR,VELT,DADOS)

l = 0.7;                % Largura do veiculo [m]
dPSI = XOUT(:,1);       % Velocidade angular [rad/s]
ALPHAT = XOUT(:,2);     % Ângulo de deriva do centro de massa [rad]
PSI = XOUT(:,3);        % Orientação [rad]
XT = XOUT(:,4);         % Posição na horizontal [m]
YT = XOUT(:,5);         % Posição na vertical [m]

ALPHAF = ALPHAFRONT;    % Ângulo de deriva na dianteira
ALPHAR = ALPHAREAR;     % Ângulo de deriva na traseira

VF = VELF;              % Módulo da velocidade no eixo dianteiro [m/s]
VR = VELR;              % Módulo da velocidade no eixo traseiro [m/s]
VT = VELT;              % Módulo da velocidade no centro de massa [m/s]

a = DADOS(3);           % Distância do eixo dianteiro ao centro de massa (F-T) [m]
b = DADOS(4);           % Distância do eixo traseiro ao centro de massa (R-T) [m]

%% Posição relativa dos cantos e eixos 
% Determina a posição dos cantos e eixos do veículo com relação ao centro
% de massa.

% Vetores posição 1, 2, 3 e 4 em relação a T na base (T t1 t2 t3)
rt1t = [a;l];           % dianteira esquerda
rt2t = [a;-l];          % dianteira direita
rt3t = [-b;-l];         % traseira direita
rt4t = [-b;l];          % traseira esquerda

eif = [a;0];            % Posicao do eixo dianteiro
eir = [-b;0];           % Posicao do eixo trasiero

%% Evolução da posição absoluta dos cantos e eixos
% Determina a movimentação dos pontos devido a mudança de orientação do
% veículo.

for j=1:length(TOUT)
    % Matriz de rotação da base (T t1 t2 t3) para (o i j k)
    RTI=[cos(PSI(j)) -sin(PSI(j));sin(PSI(j)) cos(PSI(j))];
    % Vetores posição 1, 2, 3 e 4 em relação a origem do ref inercial na
    % base (T t1 t2 t3)
    rt1i(j,1:2) = (RTI*rt1t)';
    rt2i(j,1:2) = (RTI*rt2t)';
    rt3i(j,1:2) = (RTI*rt3t)';
    rt4i(j,1:2) = (RTI*rt4t)';
    % Posicionando o eixo dianteiro e o traseiro
    eff(j,1:2) = (RTI*eif);     % Eixo dianteiro
    err(j,1:2) = (RTI*eir);     % Eixo trasiro
end

%% Posição absoluta dos cantos e eixos
% A evolução da posição absoluta dos pontos ao longo do tempo.

% Vetores posição 1, 2, 3 e 4 em relação a o na base (o i j k)
rc1t=[XT YT]+rt1i;
rc2t=[XT YT]+rt2i;
rc3t=[XT YT]+rt3i;
rc4t=[XT YT]+rt4i;

% Posicionamento absoluto do eixo dianteiro e trasiero
ef = [XT YT]+eff;
er = [XT YT]+err;

%% Ajuste do tempo
% A exibição deve ser ajustada pois a o número de frames não é a mesma que
% a resolução do integrador (TSPAN).
%

TEMPO = 0:0.05:TOUT(end);

for i=1:length(TEMPO)
    % Posição dos cantos e eixo
    rc1(i,1:2) = interp1(TOUT,rc1t,TEMPO(i));
    rc2(i,1:2) = interp1(TOUT,rc2t,TEMPO(i));
    rc3(i,1:2) = interp1(TOUT,rc3t,TEMPO(i));
    rc4(i,1:2) = interp1(TOUT,rc4t,TEMPO(i));
    efrente(i,1:2) = interp1(TOUT,ef,TEMPO(i));
    etras(i,1:2) = interp1(TOUT,er,TEMPO(i));

    % Posição do centro de massa
    xxx(i,1:2) = interp1(TOUT,XT,TEMPO(i));
    yyy(i,1:2) = interp1(TOUT,YT,TEMPO(i));

    % Estados
    alphat(i,1:2) = interp1(TOUT,ALPHAT,TEMPO(i));
    psii(i,1:2) = interp1(TOUT,PSI,TEMPO(i));

    % Ângulos de deriva
    alphaf(i,1:2) = interp1(TOUT,ALPHAF,TEMPO(i));
    alphar(i,1:2) = interp1(TOUT,ALPHAR,TEMPO(i));
    
    % Velocidade    
    velf(i,1:2) = interp1(TOUT,VF,TEMPO(i));
    velr(i,1:2) = interp1(TOUT,VR,TEMPO(i));
    velt(i,1:2) = interp1(TOUT,VT,TEMPO(i));
end

%% Definindo a figura
% Gerando a figura e definindo algumas propriedades
%

f=figure(666);
set(f,'Units','centimeters')
set(f,'Position',[1 1 34 17])
ax=gca();
hold on
axis equal

% Definindo os limites de exibição
set(ax,'XLim',[min(XT)-10 max(XT)+10])
set(ax,'XLimMode','manual')
set(ax,'YLim',[min(YT)-10 max(YT)+10])
set(ax,'YLimMode','manual')

% Legendas 
ti = title('Trajetória');
xl = xlabel('Distância [m]');
yl = ylabel('Distância [m]');

%% Primeiro frame
%
%

% Vetores velocidade
% Script "vetor.m"
vetor([xxx(1) yyy(1)],(alphat(1)+psii(1)),velt(1),'k');
vetor(efrente(1,1:2),(alphaf(1)+psii(1)),velf(1),'g');
vetor(etras(1,1:2),(alphar(1)+psii(1)),velr(1),'b');

% Coordenadas dos cantos para o primeiro frame
xc = [rc1(1,1) rc2(1,1) rc3(1,1) rc4(1,1)];
yc = [rc1(1,2) rc2(1,2) rc3(1,2) rc4(1,2)];

% Exibindo o veículo
fill(xc,yc,'r')

% frame = getframe(6);
% im = frame2im(frame);
% [A,map] = rgb2ind(im,256,'nodither'); 
% imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','LoopCount',Inf,'DelayTime',0.1);

%% Frames intermediários
%
%

for j = 1:length(TEMPO)
    % Trajetória do centro de massa
    plot(XT,YT,'r')

    % Coordenadas dos cantos para os frames
    xc = [rc1(j,1) rc2(j,1) rc3(j,1) rc4(j,1)];
    yc = [rc1(j,2) rc2(j,2) rc3(j,2) rc4(j,2)];

    % Vetores velocidade
    % Com cores diferentes
    vetor([xxx(j) yyy(j)],(alphat(j)+psii(j)),velt(j),'k');
    vetor(efrente(j,1:2),(alphaf(j)+psii(j)),velf(j),'g');
    vetor(etras(j,1:2),(alphar(j)+psii(j)),velr(j),'b');

    % Exibindo o veículo
    fill(xc,yc,'r')

    % frame = getframe(6);
    % im = frame2im(frame);
    % [A,map] = rgb2ind(im,256,'nodither'); 
    % imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','WriteMode','append','DelayTime',0.05);

    % Pausa a exibição - ATENÇÂO: Tem que ser o mesmo valor usado no ajuste
    % do tempo

    pause(0.05) 

    cla(ax); % Limpando o axes
end

%% Último frame
% A última imagem que a figura vai exibir quando a animação acabar
%

plot(XT,YT,'r')

% Coordenadas dos cantos para o último frame
xc = [rc1(end,1) rc2(end,1) rc3(end,1) rc4(end,1)];
yc = [rc1(end,2) rc2(end,2) rc3(end,2) rc4(end,2)];

% Exibindo o veículo
fill(xc,yc,'r')

end

%% Ver também
%
% <index.html Início> | <vetor.html Vetor>
%