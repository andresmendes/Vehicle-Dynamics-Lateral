%% Open-Loop Steering Simple
% This example simulates a simple vehicle with an open loop
% sinusoidal steering input.
%
% <<OpenLoopSteeringSimple.gif>>
%
%% Simulation models and parameters
% First, all classes of the package are imported with
%

clear ; close all ; clc

import VehicleDynamicsLateral.*

%% Tire and vehicle model
% Choosing tire and vehicle model. In this case, the parameters are defined
% by the user.
%

% Choosing tire
TireModel = TirePacejka();
% Choosing vehicle
System = VehicleSimpleNonlinear();

% Tire model and friction
System.tire = TireModel;
System.muy  = 1.0;

%% Simulation parameters
% Choosing simulation time span
%

T = 4;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

%% Open-loop steering input
% Steering angle
System.deltaf = 1*pi/180*sin(T^-1*2*pi*TSPAN);

%% Run simulation
% To define a simulation object (simulator) the arguments must be the
% vehicle object and the time span.

simulator = Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

%% Results

% Retrieving states
XT      = simulator.XT;
YT      = simulator.YT;
PSI     = simulator.PSI;
VEL     = simulator.VEL;
ALPHAT  = simulator.ALPHAT;
dPSI    = simulator.dPSI;

figure(1)
plot(TSPAN,180/pi*System.deltaf)
xlabel('time [s]')
ylabel('Steering angle [deg]')

%%
% Frame and animation

g = Graphics(simulator);
g.TractorColor = 'r';

g.Frame();
g.Animation();
% g.Animation('html/OpenLoopSteeringSimple');       % Uncomment to save animation gif

%% See Also
%
% <../../../index.html Home> | <../../OpenLoopSteeringArticulated/html/OpenLoopSteeringArticulated.html Open-Loop Steering Articulated>
%
