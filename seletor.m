%% Seletor
% Este script define as variáveis necessárias para as simulações de acordo
% com os modelos e dados escolhidos.
%
%% Código
% Código da função:

function [pneuFun,veiculoFun,pneuDadosFrente,pneuDadosTras,veiculoDadosVet,pneuTxt,veiculoTxt]...
    = seletor(pneuModelo,pneuDados,veiculoModelo,veiculoDados)

%% Descrição
% Esta função tem como objetivo fazer a seleção dos dados e modelos de pneu e veículo

%% Pneu
% Os dados do pneu são definidos de acordo com o modelo de pneu escolhido

% Selecionando os dados do pneu
if pneuDados == 1
	pneuLinearDados
end
%--------------------------------------------------------------------------
if pneuDados == 2
	pneuSadriDadosTaylor
end
%--------------------------------------------------------------------------
if pneuDados == 3
	pneuSadriDadosAjuste
end
%--------------------------------------------------------------------------
if pneuDados == 4
	pneuPacejkaDados
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Selecionando o modelo de pneu
if pneuModelo == 1
	pneuFun = @pneuLinearFun; % Definindo a função de pneu como a linear

	% Texto sobre o modelo de pneu para uso em descrição de gráficos 
	pneuTxt = ' linear'; 
end
%--------------------------------------------------------------------------
if pneuModelo == 2
	pneuFun = @pneuSadriFun; % Definindo a função de pneu como Sadri

	% Texto sobre o modelo de pneu para uso em descrição de gráficos 
	pneuTxt = ' Sadri'; 
end
%--------------------------------------------------------------------------
if pneuModelo == 3
	pneuFun = @pneuPacejkaFun; % Definindo a função de pneu como Pacejka

	% Texto sobre o modelo de pneu para uso em descrição de gráficos 
	pneuTxt = ' Pacejka'; 

end

if pneuModelo == 4
	pneuFun = @pneuPacejkaEstFun; % Definindo a função de pneu como Pacejka

	% Texto sobre o modelo de pneu para uso em descrição de gráficos 
	pneuTxt = ' Pacejka Estendido'; 

end

%% Veículo
% Os dados do veículo são definidos de acordo com o modelo de pneu
% escolhido

% Selecionando os dados do pneu
if veiculoDados == 1
	veiculoDadosScript
	veiculoDadosVet = veiculoDadosVetor;
end
%--------------------------------------------------------------------------
if veiculoDados == 2
	veiculoDadosScript14
	veiculoDadosVet = veiculoDadosVetor;
end
%--------------------------------------------------------------------------
if veiculoDados == 3
	veiculoDadosScript45
	veiculoDadosVet = veiculoDadosVetor;
end
%--------------------------------------------------------------------------
%--------------------------------------------------------------------------
% Modelo de veículo
if veiculoModelo == 1
	veiculoFun = @veiculoLinear2gdl; % Definindo a função de veículo
	
	% Texto sobre o modelo de veiculo para uso em descrição de gráficos 
    veiculoTxt = ' linear 2 GDL';
end

if veiculoModelo == 2
	veiculoFun = @veiculoNaoLinear2gdl; % Definindo a função de veículo
	
	% Texto sobre o modelo de veiculo para uso em descrição de gráficos 
    veiculoTxt = ' não linear 2 GDL'; 
end

if veiculoModelo == 3
	veiculoFun = @veiculoNaoLinear3gdl; % Definindo a função de veículo

	% Texto sobre o modelo de veiculo para uso em descrição de gráficos 
    veiculoTxt = ' não linear 3 GDL'; 
end

if veiculoModelo == 4
	% Definindo a função de veículo
    veiculoFun = @veiculoNaoLinear3gdlEst; 
    % Texto sobre o modelo de veiculo para uso em descrição de gráficos 
	veiculoTxt = ' não linear 3 GDL Estendido'; 
end

end

%% Ver também
%
% <index.html Início> 
%