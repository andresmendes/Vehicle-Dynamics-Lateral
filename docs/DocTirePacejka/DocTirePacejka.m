%% Pacejka tire model
% Nonlinear relationship between tire lateral force and slip angle expressed by a semi-empirical model with experimental coefficients [1].
%
% <html>
% <script src='https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML'></script>
% </html>
%
%% Sintax
% |Fy = _TireModel_.Characteristic(alpha, Fz, muy)|
%
%
% |_TireModel_.PlotTire()|
%
%% Arguments
% The following table describes the input arguments
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>alpha</tt></td> <td width="70%">Tire slip angle [rad]</td> </tr>
% <tr> <td width="30%"><tt>Fz</tt></td> <td width="70%">Vertical force [N]</td> </tr>
% <tr> <td width="30%"><tt>muy</tt></td> <td width="70%">Friction coefficient [-]</td> </tr>
% </table> </html>
%
%% Outputs
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>Fy</tt></td> <td width="70%">Tire lateral force [N]</td> </tr>
% </table> </html>
%
%% Description
% The lateral force can be written as
%
% \[ F_y = - \frac{\mu_y}{\mu_{y, n}} (F_{y, n}(\alpha_{eq}) + S_v) \]
%
% where \(\alpha_{eq}\) is the equivalent slip angle
%
% \[ \alpha_{eq} = \frac{\mu_{y0}}{\mu_y} \frac{F_{z0}}{F_z} (\alpha + S_h) \]
%
% and \(F_{y, n}\) is the reference function of the lateral force
%
% \[ F_{y, n} = D \sin(C \arctan(B \alpha - E (B \alpha - \arctan(B \alpha)))) \]
%
% The coefficients \(B\), \(C\), \(D\) and \(E\) can be written as
%
% \[ C = a_0 \]
%
% \[ D = \mu_{y, n} F_z = (a_1 F_z + a_2) F_z \]
%
% \[ B = \frac{B C D}{C D} = a_3 \sin \left\{ 2 \arctan \left( \frac{F_z}{a_4} \right) \right\} (1 - a_5 | \gamma |) \]
%
% \[ E = a_6 F_z + a_7 \]
%
% The horizontal and vertical shifts of the curve are calculated as
%
% \[ S_h = a_8 \gamma + a_9 F_z + a_{10} \]
%
% \[ S_v = a_{11} F_z \gamma + a_{12} F_z + a_{13} \]
%
% The model implemented here converts the slip angle using the following equation
%
% _ALPHA = asin(sin(alpha));_
%
% This equation alters the slip angle in such way that the characteristic equation becames symmetric in relation to the vertical line at 90 degrees and the lateral force becomes zero at 180 degrees. The same analogy can be made with negative values of the slip angle
%
% *Hypothesis*
%
% * Nonlinear tire model
% * Covers the whole range of slip angles (-180 to 180 degrees)
%
%% References
% [1] BAKKER, E.; PACEJKA, H. B.; LIDNER, L. A new tire model with an application in vehicle dynamics studies. [S.l.], 1989
%
%% See Also
%
% <../../../index.html Home>
%
