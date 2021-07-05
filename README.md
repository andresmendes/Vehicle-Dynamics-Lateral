[![View Vehicle Dynamics - Lateral on File Exchange](https://www.mathworks.com/matlabcentral/images/matlab-file-exchange.svg)](https://www.mathworks.com/matlabcentral/fileexchange/58683-vehicle-dynamics-lateral)

# Vehicle Dynamics - Lateral: Open Source Simulation Package for MATLAB

This package is an open source initiative that provides vehicle models and graphics features for lateral dynamics simulation of simple and articulated vehicles.

Vehicle Dynamics - Lateral is part of the [OpenVD Project](https://github.com/andresmendes/openvd).

![](http://andresmendes.github.io/Vehicle-Dynamics-Lateral/Examples/TemplateArticulated/html/TemplateArticulated.gif)

## Documentation

Examples, Models and Functions are here: [Matlab File Exchange: Vehicle Dynamics - Lateral](https://www.mathworks.com/matlabcentral/fileexchange/58683-vehicle-dynamics-lateral) 

* Examples - Simulation and modeling examples.
* Models - Simulink models.
* Functions - Function documentation.

To make the first steps easier, two template scripts are available covering the simulation of simple and articulated vehicles. We encourage the users to run and explore the examples **TemplateSimple.m** and **TemplateArticulated.m**.

Alternatively, for users familiar with Simulink, two template applications are available for running the models of the package in Simulink. Run and explore the examples **TemplateSimpleSimulink.m** and **TemplateArticulatedSimulink.m**.

## Description 

The general structure of the package is illustrated below. All classes of the package are categorized into Vehicle model, Tire model and Graphics. One Vehicle model and one Tire model are combined to form the System. The integration of the System, with the apropriate parameters and initial conditions, is performed through the standard [ode45](https://www.mathworks.com/help/matlab/ref/ode45.html) function of MATLAB®. The resulting data can be ploted as Frame and Animation with the Graphics features.

This package uses an object-oriented programming architecture. For more details see [Object-Oriented Programming in MATLAB](https://www.mathworks.com/products/matlab/object-oriented-programming.html).

![Structure](https://www.dropbox.com/s/xzqpgjav7h39up8/vehicle_dynamics_lateral_structure.png?raw=1)

## Installation

The first thing you have to do is install the package according to the following steps:

* Download the latest version of Vehicle Dynamics - Lateral.
* Save the package (folder "+VehicleDynamicsLateral") in the MATLAB® path or add your current path to the paths list. More details in [help path](https://www.mathworks.com/help/matlab/ref/path.html).

## API Documentation

Help and documentation on-the-fly are available through the "doc" and "help" commands, as usual. For instance, run:

<code>help VehicleDynamicsLateral.VehicleSimpleLinear</code>

## Modules

Vehicle Dynamics - Lateral (Current page)

[Vehicle Dynamics - Longitudinal](https://github.com/andresmendes/Vehicle-Dynamics-Longitudinal/)

[Vehicle Dynamics - Vertical](https://github.com/andresmendes/Vehicle-Dynamics-Vertical/)

## Other projects

Kinematic bicycle model - Open loop steering rate ([Matlab File Exchange](https://www.mathworks.com/matlabcentral/fileexchange/91910-kinematic-bicycle-model-open-loop-steering-rate) | [GitHub](https://github.com/andresmendes/Kinematic-bicycle-model---Open-loop-steering-rate))

## Publications

[Project OpenVD: Open Vehicle Dynamics](https://www.researchgate.net/project/OpenVD-Open-Vehicle-Dynamics/)
