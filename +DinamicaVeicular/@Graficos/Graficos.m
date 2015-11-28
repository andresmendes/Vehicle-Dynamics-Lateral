%% Animação
% Fazer a animação do veículo se movimentando.
%
%% Código
%
classdef Graficos
	methods
        % constructor
        function self = Graficos(varargin)
            v = DinamicaVeicular.VeiculoArticuladoNaoLinear4GDL;
            if nargin == 0
                self.veiculo = v.params;
            else
                self.veiculo = varargin{1}.params;
            end
        end

        function AnimacaoArticulado(self,XOUT,TOUT)
        l = 2.6/2;              % Metade da largura do veiculo [m] % Scania
        dPSI = XOUT(:,1);       % Velocidade angular [rad/s]
        ALPHAT = XOUT(:,2);     % Ângulo de deriva do centro de massa [rad]
        dPHI = XOUT(:,3);       % Velocidade angular relativa entre as unidades [rad/s]
        VT = XOUT(:,4);         % Módulo da velocidade no centro de massa do caminhão-trator [m/s]
        PHI = XOUT(:,5);        % Orientação relativa do semirreboque [rad]
        PSI = XOUT(:,6);        % Orientação absoluta do caminhão tratos [rad]
        XT = XOUT(:,7);         % Posição na horizontal [m]
        YT = XOUT(:,8);         % Posição na vertical [m]

        a = self.veiculo(19);        % distancia do eixo dianteiro ao centro de massa do caminhão-trator [m]
        b = self.veiculo(20);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
        c = self.veiculo(9);        % distancia da articulação ao centro de massa do caminhão-trator [m]
        d = self.veiculo(21);        % distancia do eixo traseiro ao centro de massa do caminhão-trator [m]
        e = self.veiculo(22);        % distancia da articulação ao centro de massa do caminhão-trator [m]

        % Calculando variáveis
        %

        % Ângulo de deriva na dianteira [rad]
        ALPHAF = atan2((a*dPSI + VT.*sin(ALPHAT)),(VT.*cos(ALPHAT))); % - DELTA;
        % Ângulo de deriva na traseira [rad]
        ALPHAR = atan2((-b*dPSI + VT.*sin(ALPHAT)),(VT.*cos(ALPHAT)));
        % Ângulo de deriva no eixo do semirreboque [rad]
        ALPHAM = atan2(((d + e)*(dPHI - dPSI) + VT.*sin(ALPHAT + PHI) - b*dPSI.*cos(PHI) - c*dPSI.*cos(PHI)),(VT.*cos(ALPHAT + PHI) + b*dPSI.*sin(PHI) + c*dPSI.*sin(PHI)));

        %
        % OBS: Calculando os ângulos de deriva com "atan2", quando o valor chega a
        % 180 graus fica estranho o vetor.
        %
        % Módulo da velocidade no eixo dianteiro [m/s]
        VF = sqrt((VT.*cos(ALPHAT)).^2 + (a*dPSI + VT.*sin(ALPHAT)).^2);
        % Módulo da velocidade no eixo traseiro [m/s]
        VR = sqrt((VT.*cos(ALPHAT)).^2 + (-b*dPSI + VT.*sin(ALPHAT)).^2);
        % Módulo da velocidade no eixo do semirreboque [m/s]
        VM = sqrt((VT.*cos(ALPHAT + PHI) + b*dPSI.*sin(PHI) + c*dPSI.*sin(PHI)).^2 + ((d + e)*(dPHI - dPSI) + VT.*sin(ALPHAT + PHI) - b*dPSI.*cos(PHI) - c*dPSI.*cos(PHI)).^2);

        RS = [XT-(b+c)*cos(PSI)-d*cos(PSI-PHI) YT-(b+c)*sin(PSI)-d*sin(PSI-PHI)];

        % Posição relativa dos cantos e eixos
        % Determina a posição dos cantos e eixos do veículo com relação ao centro
        % de massa.

        % Vetores posição 1, 2, 3 e 4 em relação a T na base (T t1 t2 t3)
        rt1t = [a;l];           % dianteira esquerda
        rt2t = [a;-l];          % dianteira direita
        rt3t = [-b;-l];         % traseira direita
        rt4t = [-b;l];          % traseira esquerda

        eif = [a;0];            % Posicao do eixo dianteiro
        eir = [-b;0];           % Posicao do eixo trasiero

        % Vetores posição 1, 2, 3 e 4 em relação a S na base (S s1 s2 s3)
        rs1s = [d;l];           % dianteira esquerda
        rs2s = [d;-l];          % dianteira direita
        rs3s = [-e;-l];         % traseira direita
        rs4s = [-e;l];          % traseira esquerda

        eim = [-e;0];           % Posicao do eixo do semirreboque

        % Evolução da posição absoluta dos cantos e eixos
        % Determina a movimentação dos pontos devido a mudança de orientação do
        % veículo.

% Pre alocando as matrizes

rt1i = zeros(length(TOUT),2);
rt2i = zeros(length(TOUT),2);
rt3i = zeros(length(TOUT),2);
rt4i = zeros(length(TOUT),2);

eff = zeros(length(TOUT),2);
err = zeros(length(TOUT),2);

rn1i = zeros(length(TOUT),2);
rn2i = zeros(length(TOUT),2);
rn3i = zeros(length(TOUT),2);
rn4i = zeros(length(TOUT),2);

emm = zeros(length(TOUT),2);

% Início do loop

        for j=1:length(TOUT)
        % Matriz de rotação da base (T t1 t2 t3) para (o i j k)
        RTI=[cos(PSI(j)) -sin(PSI(j));sin(PSI(j)) cos(PSI(j))];
        % Vetores posição 1, 2, 3 e 4 em relação a origem do ref inercial na
        rt1i(j,1:2) = (RTI*rt1t)';
        % base (T t1 t2 t3)
        rt2i(j,1:2) = (RTI*rt2t)';
        rt3i(j,1:2) = (RTI*rt3t)';
        rt4i(j,1:2) = (RTI*rt4t)';
        % Posicionando o eixo dianteiro e o traseiro
        eff(j,1:2) = (RTI*eif);     % Eixo dianteiro
        err(j,1:2) = (RTI*eir);     % Eixo trasiro

        % Matriz de rotação da base (S s1 s2 s3) para (o i j k)
        RSI=[cos(PSI(j)-PHI(j)) -sin(PSI(j)-PHI(j));sin(PSI(j)-PHI(j)) cos(PSI(j)-PHI(j))];
        % Vetores posição 1, 2, 3 e 4 em relação a O na base (c c1 c2 c3)
        rn1i(j,1:2) = (RSI*rs1s)';
        rn2i(j,1:2) = (RSI*rs2s)';
        rn3i(j,1:2) = (RSI*rs3s)';
        rn4i(j,1:2) = (RSI*rs4s)';

        % Posicionando o eixo dianteiro e o traseiro
        emm(j,1:2) = (RSI*eim);     % Eixo trasiro
        end

        % Posição absoluta dos cantos e eixos
        % A evolução da posição absoluta dos pontos ao longo do tempo.

        % Vetores posição 1, 2, 3 e 4 em relação a o na base (o i j k)
        rc1t=[XT YT]+rt1i;
        rc2t=[XT YT]+rt2i;
        rc3t=[XT YT]+rt3i;
        rc4t=[XT YT]+rt4i;

        % Posicionamento absoluto do eixo dianteiro e trasiero
        ef = [XT YT]+eff;
        er = [XT YT]+err;

        % Vetores posição 1, 2, 3 e 4 em relação a o na base (o i j k)
        rn1t=RS+rn1i;
        rn2t=RS+rn2i;
        rn3t=RS+rn3i;
        rn4t=RS+rn4i;

        em = RS+emm;

        % Ajuste do tempo
        % A exibição deve ser ajustada pois a o número de frames não é a mesma que
        % a resolução do integrador (TSPAN).
        %

        TEMPO = 0:0.05:TOUT(end);

% Pre alocando as matrizes
rc1 = zeros(length(TEMPO),2);
rc2 = zeros(length(TEMPO),2);
rc3 = zeros(length(TEMPO),2);
rc4 = zeros(length(TEMPO),2);

efrente = zeros(length(TEMPO),2);
etras = zeros(length(TEMPO),2);

xxx = zeros(length(TEMPO),2);
yyy = zeros(length(TEMPO),2);
alphat = zeros(length(TEMPO),2);
psii = zeros(length(TEMPO),2);
phii = zeros(length(TEMPO),2);

alphaf = zeros(length(TEMPO),2);
alphar = zeros(length(TEMPO),2);
alpham = zeros(length(TEMPO),2);

velf = zeros(length(TEMPO),2);
velr = zeros(length(TEMPO),2);
velt = zeros(length(TEMPO),2);
velm = zeros(length(TEMPO),2);

rn1 = zeros(length(TEMPO),2);
rn2 = zeros(length(TEMPO),2);
rn3 = zeros(length(TEMPO),2);
rn4 = zeros(length(TEMPO),2);

emsemi = zeros(length(TEMPO),2);


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
        phii(i,1:2) = interp1(TOUT,PHI,TEMPO(i));

        % Ângulos de deriva
        alphaf(i,1:2) = interp1(TOUT,ALPHAF,TEMPO(i));
        alphar(i,1:2) = interp1(TOUT,ALPHAR,TEMPO(i));
        alpham(i,1:2) = interp1(TOUT,ALPHAM,TEMPO(i));

        % Velocidade
        velf(i,1:2) = interp1(TOUT,VF,TEMPO(i));
        velr(i,1:2) = interp1(TOUT,VR,TEMPO(i));
        velt(i,1:2) = interp1(TOUT,VT,TEMPO(i));
        velm(i,1:2) = interp1(TOUT,VM,TEMPO(i));

        % Semirreboque
        rn1(i,1:2) = interp1(TOUT,rn1t,TEMPO(i));
        rn2(i,1:2) = interp1(TOUT,rn2t,TEMPO(i));
        rn3(i,1:2) = interp1(TOUT,rn3t,TEMPO(i));
        rn4(i,1:2) = interp1(TOUT,rn4t,TEMPO(i));

        emsemi(i,1:2) = interp1(TOUT,em,TEMPO(i));
        end

        % Definindo a figura
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
        title('Trajet\''oria','Interpreter','Latex')
        xlabel('Dist\^ancia [m]','Interpreter','Latex');
        ylabel('Dist\^ncia [m]','Interpreter','Latex');

        % Primeiro frame
        %
        %

        % Vetores velocidade
        % Script "vetor.m"
        % Escolhi não exibir o vetor velocidade do centro de massa pq fica muito poluido
        %vetor([xxx(1) yyy(1)],(alphat(1)+psii(1)),velt(1),'k');
        self.Vetor(efrente(1,1:2),(alphaf(1)+psii(1)),velf(1),'r');
        self.Vetor(etras(1,1:2),(alphar(1)+psii(1)),velr(1),'g');
        self.Vetor(emsemi(1,1:2),(alpham(1)+psii(1)-phii(1)),velm(1),'b');

        % Coordenadas dos cantos para o primeiro frame
        xc = [rc1(1,1) rc2(1,1) rc3(1,1) rc4(1,1)];
        yc = [rc1(1,2) rc2(1,2) rc3(1,2) rc4(1,2)];

        xn = [rn1(1,1) rn2(1,1) rn3(1,1) rn4(1,1)];
        yn = [rn1(1,2) rn2(1,2) rn3(1,2) rn4(1,2)];

        % Exibindo o veículo
        fill(xc,yc,'r')
        fill(xn,yn,'g')

        % % Inicializando o gif
        % frame = getframe(666);
        % im = frame2im(frame);
        % [A,map] = rgb2ind(im,256,'nodither');
        % imwrite(A,map,'.../result/gifs/animacao.gif','LoopCount',Inf,'DelayTime',0.1);

        % Frames restantes
        %

        for j = 1:length(TEMPO)
        % Trajetória do centro de massa

        % Centro de massa (Comentado porque não sei se vou usar)
        % plot(XT,YT,'r')
        % plot(RS(:,1),RS(:,2),'g')

        % Eixos
        plot(efrente(:,1),efrente(:,2),'r')
        plot(etras(:,1),etras(:,2),'g')
        plot(emsemi(:,1),emsemi(:,2),'b')

        % Coordenadas dos cantos para os frames
        xc = [rc1(j,1) rc2(j,1) rc3(j,1) rc4(j,1)];
        yc = [rc1(j,2) rc2(j,2) rc3(j,2) rc4(j,2)];
        xn = [rn1(j,1) rn2(j,1) rn3(j,1) rn4(j,1)];
        yn = [rn1(j,2) rn2(j,2) rn3(j,2) rn4(j,2)];


        % Exibindo o veículo
        fill(xc,yc,'r')
        fill(xn,yn,'g')

        % Vetores velocidade
        % Com cores diferentes
        % Escolhi não exibir o vetor velocidade do centro de massa pq fica muito poluido
        %vetor([xxx(j) yyy(j)],(alphat(j)+psii(j)),velt(j),'k');
        self.Vetor(efrente(j,1:2),(alphaf(j)+psii(j)),velf(j),'r');
        self.Vetor(etras(j,1:2),(alphar(j)+psii(j)),velr(j),'g');
        self.Vetor(emsemi(j,1:2),(alpham(j)+psii(j)-phii(j)),velm(j),'b');

        % % Adicionando o frame atual ao gif iniciado
        % frame = getframe(666);
        % im = frame2im(frame);
        % [A,map] = rgb2ind(im,256,'nodither');
        % imwrite(A,map,'../result/gifs/animacao.gif','WriteMode','append','DelayTime',0.05);

        % Pausa a exibição - ATENÇÂO: Tem que ser o mesmo valor usado no ajuste
        % do tempo

        pause(0.05)

        cla(ax); % Limpando o axes
        end

        % Último frame
        % A última imagem que a figura vai exibir quando a animação acabar
        %

        % Centro de massa
        % plot(XT,YT,'r')
        % plot(RS(:,1),RS(:,2),'g')

        % Eixos
        plot(efrente(:,1),efrente(:,2),'r')
        plot(etras(:,1),etras(:,2),'g')
        plot(emsemi(:,1),emsemi(:,2),'b')

        % Coordenadas dos cantos para o último frame
        xc = [rc1(end,1) rc2(end,1) rc3(end,1) rc4(end,1)];
        yc = [rc1(end,2) rc2(end,2) rc3(end,2) rc4(end,2)];

        xn = [rn1(end,1) rn2(end,1) rn3(end,1) rn4(end,1)];
        yn = [rn1(end,2) rn2(end,2) rn3(end,2) rn4(end,2)];



        % Exibindo o veículo
        fill(xc,yc,'r')
        fill(xn,yn,'g')

        % Escolhi não exibir o vetor velocidade do centro de massa pq fica muito poluido
        %vetor([xxx(end) yyy(end)],(alphat(end)+psii(end)),velt(end),'k');
        self.Vetor(efrente(end,1:2),(alphaf(end)+psii(end)),velf(end),'r');
        self.Vetor(etras(end,1:2),(alphar(end)+psii(end)),velr(end),'g');
        self.Vetor(emsemi(end,1:2),(alpham(end)+psii(end)-phii(end)),velm(end),'b');

        end

        % Ver também
        %
        % <index.html Início> | <vetor.html Vetor>
        %
end
methods(Static)

        function Vetor(inicio,angulo,modulo,cor)
        coord1 = inicio; % inicio do vetor
        theta = angulo;
        modulo = 0.7*modulo; % modulo do vetor
        coord2 = modulo*[cos(theta) sin(theta)] + coord1; % fim do vetor

        %theta = atan2((coord1(1)-coord2(1)),(coord1(2)-coord2(2))); % angulo de orientação do triangulo
        esc = 1; % Escala
        l = 0.5; % Largura relativa em relação ao comprimento do triangulo (0-1)

        % Forma e orientação do triangulo
        c1 = esc*l*[-sin(theta) +cos(theta)]; % canto 1 - inferior esquerdo
        c2 = esc*l*[+sin(theta) -cos(theta)]; % canto 2 - inferior direito
        c3 = esc*[+cos(theta) +sin(theta)]; % canto 3 - superior central

        % Escala e posicionamento

        x = [c1(1)+coord2(1) c2(1)+coord2(1) c3(1)+coord2(1)];
        y = [c1(2)+coord2(2) c2(2)+coord2(2) c3(2)+coord2(2)];

        % figure(1)
        % axis equal
        hold on
        fill(x,y,cor)
        p = plot([coord1(1) coord2(1)],[coord1(2) coord2(2)],cor);
        set(p,'LineWidth',2)
        % Idéia de colocar um marcador no início do vetor
        % m = plot(coord1(1),coord1(2),strcat('*',cor));
        % set(m,'MarkerSize',10)
        end

	end

    properties
        veiculo
    end
end

%% Ver também
%
% <index.html Início>
%
