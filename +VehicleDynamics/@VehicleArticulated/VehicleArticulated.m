%% Veículo articulado (Abstract)
%

classdef (Abstract) VehicleArticulated
	methods(Abstract)
		% VeiculoArticuladoNaoLinear4GDL(self,t,estados)
	end

    properties(Abstract)
		params
        tire
        distFT
        distTR
        distRA
        distAS
        distSM
        width     % Largura do caminhão-trator
        widthSemi % Largura do semirreboque
	end
end

%% Ver também
%
% <index.html Index>
%
