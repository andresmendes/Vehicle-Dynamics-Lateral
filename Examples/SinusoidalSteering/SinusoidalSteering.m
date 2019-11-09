%% Sinusoidal Steering
% Simulation of a simple vehicle with sinusoidal steering actuation.
%
%%
%

%%
%
import VehicleDynamicsLateral.*

% Choosing simulation
T = 4;                      % Total simulation time [s]
resol = 50;                 % Resolution
TSPAN = 0:T/resol:T;        % Time span [s]

% Choosing tire
TireModel = TirePacejka();
% Choosing vehicle
System = VehicleSimpleNonlinear();

% Steering angle
System.deltaf = 1*pi/180*sin(T^-1*2*pi*TSPAN);
% Tire model
System.tire = TireModel;
System.muy = 1.0;


simulator = Simulator(System, TSPAN);

% Simulation
simulator.Simulate();

%% Results

% Retrieving states
XT = simulator.XT;
YT = simulator.YT;
PSI = simulator.PSI;
VEL = simulator.VEL;
ALPHAT = simulator.ALPHAT;
dPSI = simulator.dPSI;

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

%%
% <<../illustrations/AnimationSinusoidalSteering.gif>>
%
%% See Also
%
% <../index.html Home>
%
