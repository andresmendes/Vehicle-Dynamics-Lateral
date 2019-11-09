%% Vehicle Dynamics - Lateral: Open Source Simulation Package for MATLAB
% This package is an open source initiative that provides vehicle models and graphics features for yaw dynamics simulation of simple and articulated vehicles.
%
% <<illustrations/AnimationTemplateArticulated.gif>>
%
%% Installation
% The first thing you have to do is install the package according to the following steps:
%
% * Download the Vehicle Dynamics - Lateral repository by clicking <https://github.com/andresmendes/Vehicle-Dynamics-Lateral/archive/master.zip here>
% * Save the package (folder "+VehicleDynamicsLateral") in the MATLAB(R) _path_ or add your current path to the _paths list_. More details in <http://www.mathworks.com/help/matlab/ref/path.html  help path>.
%
%% Dependencies
% To run the functions of the package and the examples from the repository a minimal MATLAB(R) version and/or Toolbox is required.
%
% Dependencies of the package (Only the files in "+VehicleDynamicsLateral"):
%

packageFilesList = getAllFiles('../+VehicleDynamicsLateral');
[packageFilesDep,packageProductDep] = matlab.codetools.requiredFilesAndProducts(packageFilesList);

packageDepNumber = length(packageProductDep);

for i = 1:packageDepNumber

    disp(strcat(packageProductDep(i).Name,', v',packageProductDep(i).Version));

end

%%
% Dependencies of the examples (Only the files in "Examples"):
%

examplesFilesList = getAllFiles('../Examples');
[examplesFilesDep,examplesProductDep] = matlab.codetools.requiredFilesAndProducts(examplesFilesList);

examplesDepNumber = length(examplesProductDep);

for i = 1:examplesDepNumber

    disp(strcat(examplesProductDep(i).Name,', v',examplesProductDep(i).Version));

end

%% Description
% The general structure of the package is illustrated below. All the classes of the package are categorized into Vehicle model, Tire model and Graphics. One Vehicle model and one Tire model are combined to form the System. The integration of the System, with the apropriate parameters and initial conditions, is performed through the standard <https://www.mathworks.com/help/matlab/ref/ode45.html ode45> function of MATLAB(R). The resulting data can be ploted as Frame and Animation with the Graphics features.
%
% This package uses an object-oriented programming architecture. For more details see <https://www.mathworks.com/discovery/object-oriented-programming.html Object-Oriented Programming in MATLAB>
%
% <<illustrations/fluxograma.svg>>
%
% The links to the description page of the available models and graphics listed below.
%
% *Tire model*
%
% * <html/DocTireLinear.html Tire linear>
% * <html/DocTirePacejka.html Tire Pacejka>
% * <html/DocTirePolynomial.html Tire polynomial>
%
% *Vehicle model*
%
% The theoretical foundation of vehicle models can be found in: <theory/vehicleSimple.pdf TheoryVehicleSimple>, <theory/vehicleSimple4DOF.pdf TheoryVehicleSimple4DOF> and <theory/vehicleArticulated.pdf TheoryVehicleArticulated>.
%
% * <html/DocVehicleArticulatedLinear.html Vehicle Articulated Linear>
% * <html/DocVehicleArticulatedNonlinear.html Vehicle Articulated Nonlinear>
% * <html/DocVehicleSimpleLinear.html Vehicle Simple Linear>
% * <html/DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear>
% * <html/DocVehicleSimpleNonlinear4DOF.html Vehicle Simple Nonlinear 4DOF>
%
% *Graphics*
%
% * <html/DocGraphics.html Graphics>
%
%% Getting started
% To make the first steps easier, two template scripts are available covering the simulation of simple and articulated vehicles. We encourage the users to run and explore the examples <html/TemplateSimple.html TemplateSimple.m> and <html/TemplateArticulated.html TemplateArticulated.m>.
%
% Alternatively, for users familiar with Simulink, two template applications are available for running the models of the package in Simulink. Run and explore the examples <html/TemplateSimpleSimulink.html TemplateSimpleSimulink.m> and <html/TemplateArticulatedSimulink.html TemplateArticulatedSimulink.m>.
%
%% Examples
% This section presents a series of studies with the successful use of the package.
%
%
%
% <html>
% <table>
%   <tr>
%     <th>Name</th>
%     <th>Description</th>
%   </tr>
%   <tr>
%     <td><a href="html/KalmanFilter.html"> Kalman Filter </a></td>
%     <td>Kalman Filter application.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SinusoidalSteering.html"> Sinusoidal Steering </a></td>
%     <td>Maneuver with sinusoidal steering angle input.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SkidPad.html"> Skid Pad </a></td>
%     <td>Simple vehicle moving in circle.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SkidPad4DOF.html"> Skid Pad 4DOF </a></td>
%     <td>Simple vehicle with roll dynamics moving in circle.</td>
%   </tr>
%   <tr>
%     <td><a href="html/SteeringControl.html"> Steering Control </a></td>
%     <td>Double lane change maneuver.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateArticulated.html"> Template Articulated </a></td>
%     <td>Articulated vehicle simulation.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateArticulatedSimulink.html"> Template Articulated Simulink </a></td>
%     <td>Simulate the articulated vehicle model in Simulink.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateSimple.html"> Template Simple </a></td>
%     <td>Simple vehicle simulation.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TemplateSimpleSimulink.html"> Template Simple Simulink </a></td>
%     <td>Simulate the simple vehicle model in Simulink.</td>
%   </tr>
%   <tr>
%     <td><a href="html/TireComparison.html"> Tire Comparison </a></td>
%     <td>Comparison of tire models.</td>
%   </tr>
% </table>
% </html>
%
%% API Documentation
% API Documentation is <html/api/api.html here>. Help and documentation on-the-fly are available through the "doc" and "help" commands, as usual.
%
%% Contributing
% There are several ways to contribute to open source projects (<https://guides.github.com/activities/contributing-to-open-source/ Contributing to open source>).
%
% To push your contribution see the following steps:
%
% * Add and/or improve Matlab files (package or examples) with codes and publishable comments.
% * Add the publish command of the new files to <html/makeDoc.html DocFiles/makeDoc.m>.
% * Create the apropriate links between the documentation pages. Ex: "See Also", "Examples", ...
% * Update index.m and api.m.
% * Run <html/makeDoc.html makeDoc.m>.
% * Copy the files from directory "Documentation" to the gh-pages branch of the repository. One easy way is using <https://github.com/davisp/ghp-import ghp-import>.
% * Commit and push.
%
%% Publications
%
% MENDES, A. S.; MENEGHETTI, D. R. ; ACKERMANN, M. ; FLEURY, A. T. .  Vehicle Dynamics - Lateral:  Open Source Simulation Package for MATLAB. In:  Congresso SAE Brasil, 2016, São Paulo. SAE Technical Paper Series, 2016.
%
%% See Also
%
% <https://github.com/andresmendes/Vehicle-Dynamics-Lateral GitHub Page>
