%% Vehicle Dynamics
% This repository is a library of functions destinated to vehicle dynamics simulation.
%
%% Instructions
% Installation steps:
%
% * Download the Vehicle-Dynamics package by clicking on "Download ZIP" in <https://github.com/andresmendes/Vehicle-Dynamics https://github.com/andresmendes/Vehicle-Dynamics>.
% * Save the package (folder "+VehicleDynamics") in the MATLAB(R) _path_ or add your current path to the _paths list_. More details in <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
% * Import the entire package with the command:
%

import VehicleDynamics.*

%% Description
%
% <<ilustracoes/fluxograma.svg>>
%
%% Templates
% The templates simulate vehicle systems according to the flowchart above.
%
% * Simple vehicle simulation: <TemplateSimple.html TemplateSimple.m>
% * Articulated vehicle simulation: <TemplateArticulated.html TemplateArticulated.m>
%
%% Tire model
% Relation between lateral force and slip angle.
%
% Tire models:
%
% * <TireLinear.html Linear Tire>
% * <TirePolynomial.html Polynomial Tire>
% * <TirePacejka1989.html Pacejka Tire>
%
% Academic:
%
% * <TireComparison.html TireComparison>
%
% Outros: <Tire.html Tire (Abstract)>
%
%% Vehicle model
% Function with the state equations of the model.
%
% Vehicle model:
%
% * <VehicleSimpleLinear2DOF.html Vehicle Simple Linear 2 DOF>
% * <VehicleSimpleNonlinear3DOF.html Vehicle Simple Nonlinear 3 DOF>
% * <VehicleArticulatedLinear4DOF.html Vehicle Articulated Linear 3 DOF> (Pendente)
% * <VehicleArticulatedNonlinear4DOF.html Vehicle Articulated Nonlinear 4 DOF>
%
% Outros: <VehicleSimple.html Vehicle Simple (Abstract)> | <VehicleArticulated.html Vehicle Articulated (Abstract)>
%
%% Graphics
% Functions for graphics generation: <Graphics.html Graphics>.
%
%% See Also
%
% <https://github.com/andresmendes/Vehicle-Dynamics https://github.com/andresmendes/Vehicle-Dynamics>
%
