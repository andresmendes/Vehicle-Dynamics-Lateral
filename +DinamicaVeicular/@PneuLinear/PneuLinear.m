classdef PneuLinear < DinamicaVeicular.Pneu
	methods
        % constructor
        function self = PneuLinear(varargin)
            if nargin == 0
                self.params = 1000;
            else
                self.params = varargin{1};
            end
        end

		function Fy = Characteristic(self,alpha)
            Fy = self.params*alpha;
        end

	end

	properties
		params
    end
end
