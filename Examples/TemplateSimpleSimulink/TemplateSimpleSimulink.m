%% Template Simple Simulink
% This template shows how to simulate an articulated vehicle in Simulink
% using a s-function. 
%
% <<TemplateSimpleSimulink.gif>>
%
%% Simulink model
% Models, parameters and initial conditions are defined in s-function.
%
%% Running Simulink model
%

clear ; close all ; clc

simulation  = sim('DiagramSimpleSimulink','StopTime','10');
simout      = simulation.simout;
tout        = simout.time;

%% Results
% 

% Retrieving vehicle model defined in s-function
[VehicleModel,~,~,~]    = SimpleVehicleSFunction(0,0,0,5);

simulator = VehicleDynamicsLateral.Simulator(VehicleModel, tout);

% Retrieving states from simulation
simulator.XT            = simout.Data(:,1);
simulator.YT            = simout.Data(:,2);
simulator.PSI           = simout.Data(:,3);
simulator.VEL           = simout.Data(:,4);
simulator.ALPHAT        = simout.Data(:,5);
simulator.dPSI          = simout.Data(:,6);

% Graphics
g = VehicleDynamicsLateral.Graphics(simulator);

g.Frame();
g.Animation();
% g.Animation('html/TemplateSimpleSimulink');       % Uncomment to save animation gif

%% See Also
%
% <../../../index.html Home> |
% <../../TemplateArticulatedSimulink/html/TemplateArticulatedSimulink.html
% Template Articulated Simulink>
%
