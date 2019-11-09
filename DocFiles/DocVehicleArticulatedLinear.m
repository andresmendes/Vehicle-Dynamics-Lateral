%% Vehicle Articulated Linear
% Linear articulated bicycle model with 4 degrees of freedom.
%
% The code of this class can be found in <api/VehicleArticulatedLinear.html VehicleArticulatedLinear>. It inherits methods from abstract class <api/VehicleArticulated.html VehicleArticulated>. The complete list of class codes is in <api/api.html API>.
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%% Theory
% The development of the equations of motion of this model can be found in <../theory/vehicleArticulated.pdf TheoryVehicleArticulated>.
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
% <tr> <td width="30%"><tt>states</tt></td> <td width="70%">Model state variables: [XT YT PSI PHI VT ALPHAT dPSI dPHI]</td> </tr>
% <tr> <td width="30%"><tt>tspan</tt></td> <td width="70%">Time span</td> </tr>
% </table> </html>
%
%% Description
% Bicycle model
%
% <<../illustrations/modelArticulatedBicycleApprox.svg>>
%
% Free body diagram
%
% <<../illustrations/modelArticulatedFreeBodyDiagram.svg>>
%
% The center of gravity of the tractor and semitrailer are located at the point \(T\) and \(S\), respectively. The front and rear axles are located at the points \(F\) and \(R\), respectively. \(A\) is the articulation point and \(M\) is the axle of the semitrailer. The constant \(a\) measures the distance of point \(F\) to \(T\) and \(b\) the distance of point \(T\) to \(R\). The distance of the articulation from the rear axle of the tractor is given by \(c\). \(d\) and \(e\) are the distances from the semitrailer. The angles \(\alpha_F\) e \(\alpha_R\) are the front and rear slip angles, respectively. \(\alpha_T\) is the vehicle side slip angle and \(\psi\) is the vehicle yaw angle. \(\delta\) is the steering angle.
%
% <<../illustrations/modelArticulated.svg>>
%
% Este modelo  escrito na forma:
%
% \[ {\bf M}({\bf x}) \dot{{\bf x}} = {\bf f}({\bf x}) \]
%
% Where \({\bf x}\) is the state vector, \({\bf M}({\bf x})\) the mass matrix and \({\bf f}({\bf x})\) is the vector function. Therefore, a function that allows the integration of the system with an explicit mass matrix is necessary. In this package the _ode45_ function is used. Details: <http://www.mathworks.com/help/matlab/ref/ode45.html?searchHighlight=%22mass%20matrix%22 ode45 (Mass matrix)>.
%
%% See Also
%
% <../index.html Home> | <DocVehicleArticulatedNonlinear.html Vehicle Articulated Nonlinear>
%
