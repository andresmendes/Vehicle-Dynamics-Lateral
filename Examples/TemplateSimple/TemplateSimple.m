%% Template Simple
% This template shows how to simulate a simple vehicle and plot the
% results.
%
% <<TemplateSimple.gif>>
%
%% Simulation models and parameters
% First, all classes of the package are imported with

clear ; close all ; clc

import VehicleDynamicsLateral.*

%%
% Choosing tire and vehicle model.

% Choosing tire
TireModel = TirePacejka();
% Choosing vehicle
VehicleModel = VehicleSimpleNonlinear();

%%
% In this case, no parameter is defined. So the default values are used.
%
% The System is completely defined once we atribute the chosen tire model
% to the vehicle object.

VehicleModel.tire = TireModel;

%%
% Choosing simulation time span

T       = 6;                        % Total simulation time         [s]
resol   = 50;                       % Resolution
TSPAN   = 0:T/resol:T;              % Time span                     [s]

%%
% To define a simulation object (simulator) the arguments must be the
% vehicle object and the time span.

simulator = Simulator(VehicleModel, TSPAN);

%%
% Changing initial conditions of the simulation object

simulator.ALPHAT0   = -0.2;         % Initial side slip angle       [rad]
simulator.dPSI0     = 0.7;          % Initial yaw rate              [rad/s]

%% Run simulation
% To simulate the system we run the Simulate method of the simulation
% object.

simulator.Simulate();

%% Results
% The time series of each state is stored in separate variables. Retrieving
% states

XT      = simulator.XT;
YT      = simulator.YT;
PSI     = simulator.PSI;
VEL     = simulator.VEL;
ALPHAT  = simulator.ALPHAT;
dPSI    = simulator.dPSI;

%%
% Plotting the states

figure(1)
hold on ; grid on ; box on
plot(TSPAN,XT)
xlabel('time [s]')
ylabel('Distance in the x direction [m]')

figure(2)
hold on ; grid on ; box on
plot(TSPAN,YT)
xlabel('time [s]')
ylabel('Distance in the y direction [m]')

figure(3)
hold on ; grid on ; box on
plot(TSPAN,PSI)
xlabel('time [s]')
ylabel('Yaw angle [rad]')

figure(4)
hold on ; grid on ; box on
plot(TSPAN,VEL)
xlabel('time [s]')
ylabel('Velocity [m/s]')

figure(5)
hold on ; grid on ; box on
plot(TSPAN,ALPHAT)
xlabel('time [s]')
ylabel('Vehicle slip angle [rad/s]')

figure(6)
hold on ; grid on ; box on
plot(TSPAN,dPSI)
xlabel('time [s]')
ylabel('Yaw rate [rad/s]')

%%
% Frame and animation

g = Graphics(simulator);
g.TractorColor = 'r';

g.Frame();
g.Animation();
% g.Animation('html/TemplateSimple');       % Uncomment to save animation gif

%% See Also
%
% <../../../index.html Home> |
% <../../TemplateArticulated/html/TemplateArticulated.html
% Template Articulated>
%
