%% Doc Generator
%

% slCharacterEncoding('windows1252')

clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% index
publish('index','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);

% Templates
publish('TemplateSimple','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',true);
    clear all                   % Clear workspace
    close all                   % Closing figures
    clc                         % Clear command window
publish('TemplateArticulated','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',true);
    clear all                   % Clear workspace
    close all                   % Closing figures
    clc                         % Clear command window

% Tire model
publish('+VehicleDynamics/@Tire/Tire','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
    publish('+VehicleDynamics/@TireLinear/TireLinear','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
    publish('+VehicleDynamics/@TirePolynomial/TirePolynomial','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
    publish('+VehicleDynamics/@TirePacejka1989/TirePacejka1989','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
publish('TireComparison','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',true);
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

% Vehicle model
publish('+VehicleDynamics/@VehicleArticulated/VehicleArticulated','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
    publish('+VehicleDynamics/@VehicleArticulatedNonlinear4DOF/VehicleArticulatedNonlinear4DOF','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
publish('+VehicleDynamics/@VehicleSimple/VehicleSimple','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
    publish('+VehicleDynamics/@VehicleSimpleNonlinear3DOF/VehicleSimpleNonlinear3DOF','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);

% Graphics
publish('+VehicleDynamics/@Graphics/Graphics','outputDir','../Vehicle-Dynamics-Documentation/','evalCode',false);
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <index.html Index>
%
