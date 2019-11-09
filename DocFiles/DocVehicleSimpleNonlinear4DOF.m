%% Vehicle Simple Nonlinear 4DOF
% Nonlinear bicycle model nonlinear with 4 degrees of freedom.
%
% The code of this class can be found in <api/VehicleSimpleNonlinear4DOF.html VehicleSimpleNonlinear4DOF>. It inherits methods from abstract class <api/VehicleSimple.html VehicleSimple>. The complete list of class codes is in <api/api.html API>.
%
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%% Theory
% The development of the equations of motion of this model can be found in <../theory/vehicleSimple4DOF.pdf TheoryVehicleSimple4DOF>.
%
%% Sintax
% |dx = _VehicleModel_.Model(t,states,tspan)|
%
% |dx = _VehicleModel_.MassMatrix(t,states,tspan)|
%
%% Arguments
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>t</tt></td> <td width="70%">Time</td> </tr>
% <tr> <td width="30%"><tt>states</tt></td> <td width="70%">Model state variables: [XT YT PSI VT ALPHAT dPSI]</td> </tr>
% <tr> <td width="30%"><tt>tspan</tt></td> <td width="70%">Time span</td> </tr>
% </table> </html>
%
%% Description
% The center of gravity of the vehicle is located at the point \(T\). The front and rear axles are located at the points \(F\) and \(R\), respectively. The constant \(a\) measures the distance of point \(F\) to \(T\) and \(b\) the distance of point \(T\) to \(R\). The angles \(\alpha_F\) e \(\alpha_R\) are the front and rear slip angles, respectively. \(\alpha_T\) is the vehicle side slip angle and \(\psi\) is the vehicle yaw angle. \(\delta\) is the steering angle.
%
% <<../illustrations/modelSimple4DOF.svg>>
%
%% See Also
%
% <../index.html Home> | <DocVehicleSimpleLinear.html Vehicle Simple Linear> | <DocVehicleSimpleNonlinear.html Vehicle Simple Nonlinear>
%
