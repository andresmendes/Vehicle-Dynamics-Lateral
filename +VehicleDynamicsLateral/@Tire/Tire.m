classdef (Abstract) Tire
    % Tire Tire abstract class
    %
    % This class must be inherited by tire models that will be used with the simulator. This class has no properties, since parameters of tire models differ significantly from one another.

    methods(Abstract)
        Characteristic(self, alpha, varargin)
    end
end

%% See Also
%
% <../../index.html Home>
%
