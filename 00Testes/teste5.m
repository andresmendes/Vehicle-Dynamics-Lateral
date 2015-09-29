theta = (0:1:360)*pi/180;

alpha = asin(sin(theta));

plot(theta*180/pi,alpha*180/pi)



v = 20;
b = 1;
dPSI = 1;
ALPHAT = (0:1:360)*pi/180;
for i = length(ALPHAT)
% Angulos de deriva não linear - Com tratamento
ALPHAR(i) = atan((v*sin(ALPHAT(i)) - b*dPSI)/(v*cos(ALPHAT(i))));         % Traseiro
if cos(ALPHAT(i))<=0
	ALPHAR(i) = -atan((v*sin(ALPHAT(i)) - b*dPSI)/(v*cos(ALPHAT(i))));
end
end

hold on

plot(ALPHAT*180/pi,ALPHAR*180/pi,'g')