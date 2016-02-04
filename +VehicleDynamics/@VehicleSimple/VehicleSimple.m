%% Veículo simples (Abstract)
%

classdef (Abstract) VehicleSimple
	methods(Abstract)
		% VeiculoArticuladoNaoLinear4GDL(self,t,estados)
	end

    properties(Abstract)
		params
        tire
        distFT
        distTR
        width
	end
end

%% See Also
%
% <index.html Index> | <VehicleArticulated.html VehicleArticulated>
%
