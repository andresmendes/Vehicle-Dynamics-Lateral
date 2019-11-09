%% Documentation Generator
%
% This script generates the whole documentation of the package.
%
%% Description
%
% The documentation of the package is written within the code of all .m files as comments.
%
% Running this script all the .html files are generated through the command <http://www.mathworks.com/help/matlab/ref/publish.html publish> and saved in the folder "../Vehicle-Dynamics-Lateral-Documentation/".
%
%% Code start

clear all                   % Clear workspace
close all                   % Closing figures
clc

docPath = '../Documentation/html/';                      % Folder where html doc files are saved
apiDocPath = strcat(docPath, 'api/');

%% Adding paths
% Adding the folder of all files to the Matlab path (only the ones that 'evalCode' is true)
%

% Examples
addpath('../Examples/KalmanFilter/')
addpath('../Examples/SinusoidalSteering/')
addpath('../Examples/SkidPad/')
addpath('../Examples/SkidPad4DOF/')
addpath('../Examples/SteeringControl/')
addpath('../Examples/TemplateArticulated/')
addpath('../Examples/TemplateArticulatedSimulink/')
addpath('../Examples/TemplateSimple/')
addpath('../Examples/TemplateSimpleSimulink/')
addpath('../Examples/TireComparison/')

%% Deleting
% Deleting old documentation

% Old gifs
delete('../Documentation/illustrations/*.gif')

% Old html
delete('../Documentation/html/*.*')
delete('../Documentation/html/api/*.*')

%% Publishing documentation
%

% Index
publish('index.m', 'outputDir', '../Documentation', 'evalCode', true,'showCode',false);

% Tire models
publish('DocTireLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocTirePacejka.m', 'outputDir', docPath, 'evalCode', false);
publish('DocTirePolynomial.m', 'outputDir', docPath, 'evalCode', false);

% Vehicle models
publish('DocVehicleArticulatedLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleArticulatedNonlinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleSimpleLinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleSimpleNonlinear.m', 'outputDir', docPath, 'evalCode', false);
publish('DocVehicleSimpleNonlinear4DOF.m', 'outputDir', docPath, 'evalCode', false);

% Graphics
publish('DocGraphics.m', 'outputDir', docPath, 'evalCode', false);

% API
publish('api.m', 'outputDir', apiDocPath,'evalCode', false);

publish('../+VehicleDynamicsLateral/@VehicleSimple/VehicleSimple.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulated/VehicleArticulated.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@VehicleSimpleLinear/VehicleSimpleLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleSimpleNonlinear/VehicleSimpleNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleSimpleNonlinear4DOF/VehicleSimpleNonlinear4DOF.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulatedLinear/VehicleArticulatedLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@VehicleArticulatedNonlinear/VehicleArticulatedNonlinear.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Tire/Tire.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TireLinear/TireLinear.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TirePolynomial/TirePolynomial.m', 'outputDir', apiDocPath, 'evalCode', false);
publish('../+VehicleDynamicsLateral/@TirePacejka/TirePacejka.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Simulator/Simulator.m', 'outputDir', apiDocPath, 'evalCode', false);

publish('../+VehicleDynamicsLateral/@Graphics/Graphics.m', 'outputDir', apiDocPath, 'evalCode', false);

% EXAMPLES
% Kalman Filter
publish('KalmanFilter.m', 'outputDir', docPath, 'evalCode', true);
close all
clearvars -except docPath apiDocPath

% Sinusoidal Steering
publish('SinusoidalSteering.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationSinusoidalSteering');
close all
clearvars -except docPath apiDocPath

% Skid Pad
publish('SkidPad.m', 'outputDir', docPath, 'evalCode', true,'showCode',true);
g.Animation('../Documentation/illustrations/AnimationSkidPad');
close all
clearvars -except docPath apiDocPath

% Skid Pad 4DOF
publish('SkidPad4DOF.m', 'outputDir', docPath, 'evalCode', true,'showCode',true);
g.Animation('../Documentation/illustrations/AnimationSkidPad4DOF');
close all
clearvars -except docPath apiDocPath

% SteeringControl
publish('SteeringControl.m', 'outputDir', docPath, 'evalCode', true,'showCode',false);
g.Animation('../Documentation/illustrations/AnimationSteeringControl');
close all
clearvars -except docPath apiDocPath

% TemplateArticulated
publish('TemplateArticulated.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationTemplateArticulated');
close all
clearvars -except docPath apiDocPath

% Template Articulated Simulink
publish('ArticulatedVehicleSFunction.m', 'outputDir', docPath, 'evalCode', false);
publish('TemplateArticulatedSimulink.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationTemplateArticulatedSimulink');
close all
clearvars -except docPath apiDocPath

% TemplateSimple
publish('TemplateSimple.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationTemplateSimple');
close all
clearvars -except docPath apiDocPath

% Template Simple Simulink
publish('SimpleVehicleSFunction.m', 'outputDir', docPath, 'evalCode', false);
publish('TemplateSimpleSimulink.m', 'outputDir', docPath, 'evalCode', true);
g.Animation('../Documentation/illustrations/AnimationTemplateSimpleSimulink');
close all
clearvars -except docPath apiDocPath

% TireComparison
publish('TireComparison.m', 'outputDir', docPath, 'evalCode', true);
close all
clearvars -except docPath apiDocPath

% DocGen
publish('makeDoc', 'outputDir', docPath, 'evalCode', false);

%% Code end
clear all                   % Clear workspace
close all                   % Closing figures
clc                         % Clear command window

%% See Also
%
% <../index.html Home>
%
