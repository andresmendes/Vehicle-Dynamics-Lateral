clear,clc
%% Descrição
% Ajuste do ângulo de deriva do centro de massa

ALPHATe = -2*2*pi:0.1:2*2*pi; % Ângulo de deriva de entrada

for i = 1:length(ALPHATe)
    
    met = sin(ALPHATe(i)); 

    if met > 0 % Verifica se esta em quadrantes da esquerda ou direita
        ALPHAT(i) = ALPHATe(i) - floor(ALPHATe(i)/(pi))*(pi);
    else
        ALPHAT(i) = ALPHATe(i) - floor(ALPHATe(i)/(pi))*(pi) - pi;
    end


end

figure(1)
plot(ALPHATe*180/pi,ALPHAT*180/pi)
xlabel('ALPHAT de entrada [grau]')
ylabel('ALPHAT de saída [grau]')
title('Conversão do ALPHAT')