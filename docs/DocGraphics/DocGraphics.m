%% Graphics
%
%% Animation
% Animates the manoever.
%
% *Sintax*
%
% |_GraphicsClass_.Animation(XOUT, TOUT, saveit)|
%
% *Arguments*
%
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>XOUT</tt></td> <td width="70%">Solution array. Each column corresponds to the solution of each state variable of the vehicle. The columns must respect the following variable order: [XT YT PSI dPSI VT ALPHAT (PHI dPHI)] </td> </tr>
% <tr> <td width="30%"><tt>TOUT</tt></td> <td width="70%">Column vector of time points.</td> </tr>
% <tr> <td width="30%"><tt>saveit</tt></td> <td width="70%">Flag for saving the animation in a gif file. If savit = 0 the animation will not be saved. If savit = 1 a file Animacao.gif is generated.</td> </tr>
% </table> </html>
%
% *Description*
%
%% Frame
% Plots the sequence of frames of the vehicle manoever.
%
% *Sintax*
%
% |_GraphicsClass_.Frame(XOUT, TOUT, saveit)|
%
% *Arguments*
%
% The following table describes the input arguments:
%
% <html> <table border=1 width="97%">
% <tr> <td width="30%"><tt>XOUT</tt></td> <td width="70%">Solution array. Each column corresponds to the solution of each state variable of the vehicle. The columns must respect the following variable order: [XT YT PSI dPSI VT ALPHAT (PHI dPHI)] </td> </tr>
% <tr> <td width="30%"><tt>TOUT</tt></td> <td width="70%">Column vector of time points.</td> </tr>
% <tr> <td width="30%"><tt>saveit</tt></td> <td width="70%">Flag for saving the trajectory image in a pdf file. If savit = 0 the image will not be saved. If savit = 1 a file Trajectory.pdf is generated.</td> </tr>
% </table> </html>
%
% *Description*
%
%% See Also
%
% <../../../index.html Home>
%
