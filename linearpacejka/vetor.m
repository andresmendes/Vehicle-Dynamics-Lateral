function vetor(inicio,angulo,modulo,cor)
% Esta função tem como objetivo plotar no gráfico com handle um vetor a
% partir da coordenada do inicio e o angulo de orientacao

coord1 = inicio; % inicio do vetor
theta = angulo; 
mod = 0.7*modulo; % modulo do vetor
coord2 = mod*[cos(theta) sin(theta)] + coord1; % fim do vetor

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

end

