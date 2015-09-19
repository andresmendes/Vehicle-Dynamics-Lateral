function animacao(XOUT,TOUT,ALPHAFRONT,ALPHAREAR,VELF,VELR,VELT,DADOS)
%% FAZER ANIMAÇÃO

l = 0.7;    % Largura do veiculo para animação [m]
dPSI = XOUT(:,1);
ALPHAT = XOUT(:,2); 
PSI = XOUT(:,3);
XT = XOUT(:,4);
YT = XOUT(:,5);

ALPHAF = ALPHAFRONT;
ALPHAR = ALPHAREAR;

VF = VELF;
VR = VELR;
VT = VELT;

a = DADOS(3);
b = DADOS(4);



    % Vetores posi��o 1, 2, 3 e 4 em rela��o a T na base (T t1 t2 t3)
    rt1t = [a;l]; % dianteira esquerda
    rt2t = [a;-l]; % dianteira direita
    rt3t = [-b;-l]; % traseira direita
    rt4t = [-b;l]; % traseira esquerda

    eif = [a;0]; % Posicao do eixo dianteiro
    eir = [-b;0]; % Posicao do eixo trasiero
for j=1:length(TOUT)
% Matriz de rota��o da base (T t1 t2 t3) para (o i j k)
RTI=[cos(PSI(j)) -sin(PSI(j));sin(PSI(j)) cos(PSI(j))];
% Vetores posi��o 1, 2, 3 e 4 em rela��o a origem do ref inercial
% na base (T t1 t2 t3)
rt1i(j,1:2) = (RTI*rt1t)';
rt2i(j,1:2) = (RTI*rt2t)';
rt3i(j,1:2) = (RTI*rt3t)';
rt4i(j,1:2) = (RTI*rt4t)';
% Posicionando o eixo dianteiro e o traseiro
eff(j,1:2) = (RTI*eif); % Eixo dianteiro
err(j,1:2) = (RTI*eir); % Eixo trasiro
end



% Vetores posi��o 1, 2, 3 e 4 em rela��o a o na base (o i j k)
rc1t=[XT YT]+rt1i;
rc2t=[XT YT]+rt2i;
rc3t=[XT YT]+rt3i;
rc4t=[XT YT]+rt4i;

% Posicionamento absoluto do eixo dianteiro
ef = [XT YT]+eff;
er = [XT YT]+err;

% Ajuste do tempo

TEMPO = 0:0.05:TOUT(end);

for i=1:length(TEMPO)

rc1(i,1:2) = interp1(TOUT,rc1t,TEMPO(i));
rc2(i,1:2) = interp1(TOUT,rc2t,TEMPO(i));
rc3(i,1:2) = interp1(TOUT,rc3t,TEMPO(i));
rc4(i,1:2) = interp1(TOUT,rc4t,TEMPO(i));

% Para o vetor vt
xxx(i,1:2) = interp1(TOUT,XT,TEMPO(i));
yyy(i,1:2) = interp1(TOUT,YT,TEMPO(i));
alphat(i,1:2) = interp1(TOUT,ALPHAT,TEMPO(i));
psii(i,1:2) = interp1(TOUT,PSI,TEMPO(i));

% Para os vetores vf e vr
alphaf(i,1:2) = interp1(TOUT,ALPHAF,TEMPO(i));
alphar(i,1:2) = interp1(TOUT,ALPHAR,TEMPO(i));
efrente(i,1:2) = interp1(TOUT,ef,TEMPO(i));
etras(i,1:2) = interp1(TOUT,er,TEMPO(i));

velf(i,1:2) = interp1(TOUT,VF,TEMPO(i));
velr(i,1:2) = interp1(TOUT,VR,TEMPO(i));
velt(i,1:2) = interp1(TOUT,VT,TEMPO(i));

end

f=figure(9);
set(f,'Units','centimeters')
set(f,'Position',[1 1 34 17])

ax=gca();
hold on
axis equal
% set(ax,'Units','centimeters')
% set(ax,'OuterPosition',[1 1 FEa*escX*FEf FEa*escY*FEf])
set(ax,'XLim',[min(XT)-10 max(XT)+10])
set(ax,'XLimMode','manual')
set(ax,'YLim',[min(YT)-10 max(YT)+10])
set(ax,'YLimMode','manual')

% axis([min(XT)-10 max(XT)+10 min(YT)-10 max(YT)+10]);
% axis manual

ti = title('Trajet\''{o}ria - Modelo n\~{a}o linear');
    set(ti,'Interpreter','Latex','FontSize',15); 
xl = xlabel('Dist\^{a}ncia [m]');
    set(xl,'Interpreter','Latex','FontSize',15);
yl = ylabel('Dist\^{a}ncia [m]');
    set(yl,'Interpreter','Latex','FontSize',15); 
    %legend('Tractor','Semitrailer')

% Primeiro frame
xc = [rc1(1,1) rc2(1,1) rc3(1,1) rc4(1,1)];
yc = [rc1(1,2) rc2(1,2) rc3(1,2) rc4(1,2)];

% Vetores
vetor([xxx(1) yyy(1)],(alphat(1)+psii(1)),velt(1),'k');
vetor(efrente(1,1:2),(alphaf(1)+psii(1)),velf(1),'g');
vetor(etras(1,1:2),(alphar(1)+psii(1)),velr(1),'b');


%hold on
fill(xc,yc,'r')



% frame = getframe(6);
% im = frame2im(frame);
% [A,map] = rgb2ind(im,256,'nodither'); 
% imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','LoopCount',Inf,'DelayTime',0.1);

for j = 1:length(TEMPO)
   
plot(XOUT(:,4),XOUT(:,5),'r')

xc = [rc1(j,1) rc2(j,1) rc3(j,1) rc4(j,1)];
yc = [rc1(j,2) rc2(j,2) rc3(j,2) rc4(j,2)];

% Vetores
vetor([xxx(j) yyy(j)],(alphat(j)+psii(j)),velt(j),'k');
vetor(efrente(j,1:2),(alphaf(j)+psii(j)),velf(j),'g');
vetor(etras(j,1:2),(alphar(j)+psii(j)),velr(j),'b');

%hold on
fill(xc,yc,'r')

% frame = getframe(6);
% im = frame2im(frame);
% [A,map] = rgb2ind(im,256,'nodither'); 
% imwrite(A,map,'/home/andre/Copy/FEI/Mestrado/Gifs/animacao.gif','WriteMode','append','DelayTime',0.05);


%    drawnow
%    F(j) = getframe;
    pause(0.05)
 %   ax=gca();
    cla(ax); %clear axes

end

% �ltimo frame

plot(XOUT(:,4),XOUT(:,5),'r')

xc = [rc1(end,1) rc2(end,1) rc3(end,1) rc4(end,1)];
yc = [rc1(end,2) rc2(end,2) rc3(end,2) rc4(end,2)];

fill(xc,yc,'r')
end