%% Multi-axle steering simple
% This script presents open-loop multi-axle steering of an simple vehicle.
%
% <<MultiaxleSteeringSimple.gif>>
%
%% Simulation models and parameters
% First, all classes of the package are imported with

clear ; close all ; clc

import VehicleDynamicsLateral.*

%%
% Choosing tire and vehicle model. In this case, the parameters are defined
% by the user.

% Choosing tire
TireModel           = TirePacejka();

% Choosing vehicle
VehicleModel        = VehicleSimpleNonlinear();
VehicleModel.tire   = TireModel;

%% Simulation parameters
% Choosing simulation time span
%

T       = 4;                            % Total simulation time     [s]
resol   = 80;                           % Resolution
TSPAN   = 0:T/resol:T;                  % Time span                 [s]

%% Open-loop steering input
% Single period sine wave with:
%

T_period            = 2;                % Steering single period    [s]
freq                = 1/T_period;       % Steering frequency        [Hz]
delta_freq          = freq*2*pi;        % Steering frequency        [rad/s]

% End input index 
% sine_index          = find(TSPAN > T_period);

% Defining steering input
VehicleModel.deltaf = 15*pi/180*sin(delta_freq*TSPAN);
VehicleModel.deltar = 15*pi/180*sin(delta_freq*TSPAN);

figure
subplot(2,1,1)
    hold on ; grid on ; box on
    plot(TSPAN,VehicleModel.deltaf*180/pi,'r','linewidth',2)
    ylabel('Delta F [deg]')
subplot(2,1,2)
    hold on ; grid on ; box on
    plot(TSPAN,VehicleModel.deltar*180/pi,'r','linewidth',2)
    ylabel('Delta R [deg]')
    xlabel('Time [s]')

%%
% To define a simulation object (simulator) the arguments must be the
% vehicle object and the time span.

simulator = Simulator(VehicleModel, TSPAN);

%%
% Initial conditions
% Changing initial conditions of the simulation object

simulator.V0 = 60/3.6;              % Initial velocity              [m/s]

%% Run simulation
% To simulate the system we run the Simulate method of the simulation
% object.

simulator.Simulate();

%% Results
%

g = Graphics(simulator);
g.Frame();
g.Animation();
% g.Animation('html/MultiaxleSteeringSimple');       % Uncomment to save animation gif

%% See Also
%
% <../../../index.html Home>
%
