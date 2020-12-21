%% Linear tire model
% Linear relationship between tire lateral force and slip angle.
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%% Sintax
% The class LinearTire has two methods
%
% |Fy = _TireModel_.Characteristic(alpha)|
%
% |_TireModel_.PlotTire()|
%
%% Arguments
% The following table describes the input arguments
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% </table> </html>
%
%% Outputs
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>Fy</tt></td> <td width="70%">Tire lateral force [N]</td> </tr>
% </table> </html>
%
%% Description
%
% The lateral force of the tire can be calculated as
%
% \[ F_y = K \alpha \]
%
% where \(F_y\) is the lateral force, \(K\) is the cornering stiffness and \(\alpha\) is the tire slip angle.
%
% *Hypothesis*
%
% * Linear tire model
% * Valid only for small values of slip angle
%
%% See Also
%
% <../../../index.html Home>
%
