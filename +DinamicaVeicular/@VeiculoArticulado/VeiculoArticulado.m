%% Veículo articulado (Abstract)
%

classdef (Abstract) VeiculoArticulado
	methods(Abstract)
		% VeiculoArticuladoNaoLinear4GDL(self,t,estados)
	end

    properties(Abstract)
		params
        pneu
        distFT
        distTR
        distRA
        distAS
        distSM
        largura     % Largura do caminhão-trator
        larguraSemi % Largura do semirreboque
	end
end

%% Ver também
%
% <index.html Início>
%
