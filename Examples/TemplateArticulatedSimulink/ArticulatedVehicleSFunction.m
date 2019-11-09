function [sys,x0,str,ts] = ArticulatedVehicleSFunction(t,x,u,flag)
% This file is a s-function template for simulating the articulated vehicle model in Simulink.

% Choosing tire model
TireModel = VehicleDynamicsLateral.TirePacejka();
% Defining tire parameters
TireModel.a0 = 1;
TireModel.a1 = 2;
TireModel.a2 = 700;
TireModel.a3 = 5000;
TireModel.a4 = 80;
TireModel.a5 = 0;
TireModel.a6 = 0;
TireModel.a7 = 0.6;
TireModel.a8 = 0;
TireModel.a9 = 0;
TireModel.a10 = 0;
TireModel.a11 = 0;
TireModel.a12 = 0;
TireModel.a13 = 0;

% Choosing vehicle model
VehicleModel = VehicleDynamicsLateral.VehicleArticulatedNonlinear();
% Defining vehicle parameters
VehicleModel.mF0 = 5200;
VehicleModel.mR0 = 2400;
VehicleModel.mF = 6000;
VehicleModel.mR = 10000;
VehicleModel.mM = 17000;
VehicleModel.IT = 46000;
VehicleModel.IS = 450000;
VehicleModel.lT = 3.5;
VehicleModel.lS = 7.7;
VehicleModel.c = -0.3;
VehicleModel.nF = 2;
VehicleModel.nR = 4;
VehicleModel.nM = 8;
VehicleModel.wT = 2.6;
VehicleModel.wS = 2.4;
VehicleModel.muy = 0.8;
VehicleModel.deltaf = 0;
VehicleModel.Fxf = 0;
VehicleModel.Fxr = 0;
VehicleModel.Fxm = 0;
VehicleModel.tire = TireModel;

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes();

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,VehicleModel);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,VehicleModel);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

% Definitions
sizes = simsizes;
sizes.NumContStates  = 8;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 8;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);

% Setting initial conditions
x0  = [0 0 0 0 15 0 0 0];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%


function sys = mdlDerivatives(t,x,u,vehicle)

% Defining input
vehicle.deltaf = u(1);
vehicle.Fxf = u(2);
vehicle.Fxr = u(3);
vehicle.Fxm = u(4);

% Getting the vehicle model function (state equations)
ModelFunction = @vehicle.Model;
% Getting the mass matrix function of the model
MassMatrixFunction = @vehicle.MassMatrix;

% The derivatives are calculated with the inverse of the MassMatrix
sys = MassMatrixFunction(t,x)\ModelFunction(t,x,0);

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(~,x,~,~)

% Output are all state variables
sys = x;

% end mdlOutputs

%% See Also
%
% <../index.html Home> | <TemplateArticulatedSimulink.html Template Articulated Simulink>
%
