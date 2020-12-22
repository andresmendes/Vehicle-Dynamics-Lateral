%% Vehicle Articulated Nonlinear Modeling
% This scipt derives the equations of motion from the nonlinear articulated
% vehicle model.
%
% For class documentation, run:
%
% |doc VehicleDynamicsLateral.VehicleArticulatedNonlinear|
%
% <html>
% <!--
% MathJax
% Source: http://docs.mathjax.org/en/latest/start.html
% -->
% <script type="text/javascript" async
% src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.5/MathJax.js?config=TeX-MML-AM_CHTML">
% </script>
% <!--
% Automatic equation numbering
% Source: http://docs.mathjax.org/en/latest/tex.html#automatic-equation-numbering
% -->
% <script type="text/x-mathjax-config">
% MathJax.Hub.Config({
% TeX: { equationNumbers: { autoNumber: "AMS" } }
% });
% </script>
% </html>
%
%% Bicycle model
% <html> 
% <p>The single track (bicycle) approximation is illustrated below</p>
% <p><img vspace="5" hspace="5" src="modelArticulatedBicycleApprox.svg" alt=""> </p>
% <p>The articulated vehicle model is illustrated below</p>
% <p><img vspace="5" hspace="5" src="modelArticulated.svg" alt=""> </p>
% <p>The vector basis \(\Omega_{\rm O} = \{ {\rm O} {\bf i} {\bf j} {\bf k} \}\) is fixed to the inertial reference frame. The vector basis \(\Omega_{\rm T} = \{ {\rm T} {\bf t}_x {\bf t}_y {\bf t}_z \}\) is fixed to the tractor and the vector basis \(\Omega_{\rm S} = \{ {\rm S} {\bf s}_x {\bf s}_y {\bf s}_z \}\) is fixed to the semitrailer. The vector basis \(\{ {\rm F} {\bf e}_x {\bf e}_y {\bf e}_z \}\) is fixed to the front axle.</p>
% <p>The center of gravity of the tractor and semitrailer are located at points \(\rm T\) and \(\rm S\), respectively. Coordinates \(x\) and \(y\) locates point \(\rm T\). The front and rear axles are located at points \(\rm F\) and \(\rm R\), respectively. \(\rm A\) is the articulation point and \(\rm M\) is the axle of the semitrailer. The constant \(a\) measures the distance of point \(\rm F\) to \(\rm T\) and \(b\) the distance of point \(\rm T\) to \(\rm R\). The distance of the articulation from the rear axle of the tractor is given by \(c\). \(d\) and \(e\) locates the position of the CG from the semitrailer. The angles \(\alpha_{\rm F}\) e \(\alpha_{\rm R}\) are the front and rear slip angles, respectively. \(\alpha_{\rm T}\) is the vehicle side slip angle, \(\psi\) is the yaw angle of the tractor and \(\phi\) is the yaw angle of the semitrailer relative to the tractor. \(\delta_{\rm F}\) is the steering angle.</p>
% </html>
%
%% Initialization
% Defining Symbols (sym) and Symbolic Functions (symfun). The subscript t
% indicates time symbolic functions.
%
% See
% <https://www.mathworks.com/help/symbolic/create-symbolic-functions.html create-symbolic-functions>.
%

clear ; close all ; clc

% Generalized coordinates (symfun)
syms x_t(t) y_t(t) PSI_t(t) PHI_t(t)
% Generalized coordinates and time derivatives (sym)
syms x dx ddx y dy ddy PSI dPSI ddPSI PHI dPHI ddPHI

% Alternative velocity related variables: Side slip angle and speed
% Alternative variables (symfun)
syms ALPHAT_t(t) VT_t(t)
% Alternative variables and time derivatives (sym)
syms VT ALPHAT dVT dALPHAT

% Inputs 
syms deltaf deltar deltam FxF FxR FxM
% Tire lateral forces
syms FyF FyR FyM
% Vehicle parameters 
syms a b c d e mT mS IT IS

%% Generalized coordinates and time derivatives
% <html>
% <p>The generalized coordinates are</p>
% <p>
% \begin{eqnarray}
% \nonumber q_1 &=& x     \\
% \nonumber q_2 &=& y     \\
% \nonumber q_3 &=& \psi  \\
% \nonumber q_4 &=& \phi,
% \end{eqnarray}
% </p>
% <p>where \(x\) and \(y\) are the coordinates of the CG of the vehicle.
% \(\psi\) is the yaw angle of the tractor and \(\phi\) is the yaw angle of
% the semitrailer relative to the tractor.</p> 
% </html>

% symfun
q_t     = [x_t y_t PSI_t PHI_t];
dq_t    = diff(q_t,t);
ddq_t   = diff(diff(q_t,t));
% sym
q       = [ x   y   PSI   PHI   ];
dq      = [ dx  dy  dPSI  dPHI  ];
ddq     = [ ddx ddy ddPSI ddPHI ];

%% Alternative velocity related variables 
% <html>
% <p>However, in many occasions it is more convenient to use the states \(v_{\rm T}\) e \(\alpha_{\rm T}\) instead of \(\dot{x}\) e \(\dot{y}\).</p>
% <p>The equations relating this pairs of variables is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \dot{x} &=& v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
%     \dot{y} &=& v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \label{eq:dxdy2vtalphat}
%     \end{eqnarray}
% </p>
% </html>

altVar_t    = [ALPHAT_t(t) VT_t(t)];
altVarD1_t  = diff(altVar_t,t);
altVar      = [ALPHAT VT];
altVarD1    = [dALPHAT dVT];

%% Conversion variables
% Array to switch symfun <-> sym

symfunVariables = [q_t  dq_t    ddq_t   altVar_t    altVarD1_t  ];
symVariables    = [q    dq      ddq     altVar      altVarD1    ];

%% Position
% <html>
% <p>The position of the CG of the tractor relative to the origin \({\rm O}\) is</p>
% <p>
% \begin{equation*} {\bf p}_{{\rm T}/{\rm O}} = x \, {\bf i} + y \, {\bf j}. \end{equation*}
% </p>
% <p>Relative positions are</p>
% <p>
% \begin{eqnarray}
% \nonumber {\bf p}_{{\rm F}/{\rm T}} &=& a \cos \psi {\bf i} + a \sin \psi {\bf j}. \\
% \nonumber {\bf p}_{{\rm R}/{\rm T}} &=& - b \cos \psi {\bf i} - b \sin \psi {\bf j}. \\
% \nonumber {\bf p}_{{\rm A}/{\rm T}} &=& - (b + c) \cos \psi {\bf i} - (b + c) \sin \psi {\bf j}. \\
% \nonumber {\bf p}_{{\rm S}/{\rm A}} &=& - d \cos \left( \psi-\phi \right) {\bf i} - d \sin \left( \psi-\phi \right) {\bf j}. \\
% \nonumber {\bf p}_{{\rm M}/{\rm A}} &=& - (d + e) \cos \left( \psi-\phi \right) {\bf i} - (d + e) \sin \left( \psi-\phi \right) {\bf j}.
% \end{eqnarray}
% </p>
% <p>The position of the CG of the semitrailer relative to the origin \({\rm O}\) is</p>
% <p>
% \begin{equation*}{\bf p}_{{\rm S}/{\rm O}} = {\bf p}_{{\rm S}/{\rm A}} + {\bf p}_{{\rm A}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left[ x - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] {\bf i} + \left[ y - \left( b + c \right) \sin \psi - d \sin \left( \psi - \phi \right) \right] {\bf j}.\end{equation*}
% </p>
% <p>Position of axles</p>
% <p>
% \begin{eqnarray}
% \nonumber {\bf p}_{{\rm F}/{\rm O}} &=& {\bf p}_{{\rm F}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left( x + a \cos \psi \right) {\bf i} + \left( y + a \sin \psi \right) {\bf j} \\
% \nonumber {\bf p}_{{\rm R}/{\rm O}} &=& {\bf p}_{{\rm F}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left( x - b \cos \psi \right) {\bf i} + \left( y - b \sin \psi \right) {\bf j} \\
% \nonumber {\bf p}_{{\rm M}/{\rm O}} &=& {\bf p}_{{\rm M}/{\rm A}} + {\bf p}_{{\rm A}/{\rm T}} + {\bf p}_{{\rm T}/{\rm O}} = \left[ x - \left( b + c \right) \cos \psi - \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf i} + \left[ y - \left( b + c \right) \sin \psi - \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf j}.
% \end{eqnarray}
% </p>
% </html>

% CG position tractor
PTO_t = [x_t y_t 0];                                            % P_{T/O}

% Relative positions
PFT_t = [ a*cos(PSI_t)  a*sin(PSI_t) 0];                        % P_{F/T}
PRT_t = [-b*cos(PSI_t) -b*sin(PSI_t) 0];                        % P_{R/T}
PAT_t = [-(b+c)*cos(PSI_t) -(b+c)*sin(PSI_t) 0];                % P_{A/T}
PSA_t = [-d*cos(PSI_t-PHI_t) -d*sin(PSI_t-PHI_t) 0];            % P_{S/A}
PMA_t = [-(d+e)*cos(PSI_t-PHI_t) -(d+e)*sin(PSI_t-PHI_t) 0 ];   % P_{M/A}

% CG position semitrailer
PSO_t = PSA_t + PAT_t + PTO_t;                                  % P_{S/O}

% Axle position
PFO_t = PFT_t + PTO_t;                                          % P_{F/O}
PRO_t = PRT_t + PTO_t ;                                         % P_{R/O}
PMO_t = PMA_t + PAT_t + PTO_t;                                  % P_{M/O}

PFO   = subs(PFO_t,symfunVariables,symVariables);               % P_{F/O}
PRO   = subs(PRO_t,symfunVariables,symVariables);               % P_{R/O}
PMO   = subs(PMO_t,symfunVariables,symVariables);               % P_{M/O}

%% Velocity 
%
% <html>
% <p>The angular velocity of the tractor and the angular velocity of the semitrailer are</p>
% <p>
%     \begin{eqnarray}
%     {\bf w}_{\rm T} &=& \dot{\psi} {\bf k}. \label{eq:tractorvelangular} \\
%     {\bf w}_{\rm S} &=& \left( \dot{\psi} - \dot{\phi} \right) {\bf k}. \label{eq:semitrailervelangular}
%     \end{eqnarray}
% </p>
% <p>Tractor and semitrailer velocities are</p>
% <p>
%     \begin{eqnarray}
%     {\bf v}_{\rm T} &=& \dot{x} {\bf i} + \dot{y} {\bf j} \label{eq:tractorveltranslation} \\
%     {\bf v}_{\rm S} &=& \left[ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] {\bf j}. \label{eq:semitrailerveltranslation}
%     \end{eqnarray}
% </p>
% <p>Velocities of each axle are</p>
% <p>
%     \begin{eqnarray}
%     {\bf v}_{\rm F} &=& \left( \dot{x} - a \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} + a \dot{\psi} \cos \psi \right) {\bf j} \label{eq:velfront} \\
%     {\bf v}_{\rm R} &=& \left( \dot{x} + b \dot{\psi} \sin \psi \right) {\bf i} + \left( \dot{y} -b \dot{\psi} \cos \psi \right) {\bf j} \label{eq:velrear} \\
%     {\bf v}_{\rm S} &=& \left[ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] {\bf j}. \label{eq:velsemitrailer}
%     \end{eqnarray}
% </p>
% </html>
%

% Angular velocity
wT_t        = [0 0 diff(PSI_t,t)];
wS_t        = [0 0 diff(PSI_t,t)-diff(PHI_t,t)];

% Velocity truck CG (T)
VTdiff_t    = diff(PTO_t,t);
VTdiff      = subs(VTdiff_t,symfunVariables,symVariables);

% Velocity semitrailer CG (S)
vSdiff_t    = diff(PSO_t,t);
vSdiff      = subs(vSdiff_t,symfunVariables,symVariables);

% Velocity truck - Alternative definition
dx_Alt_t    = VT_t*cos(PSI_t + ALPHAT_t);
dy_Alt_t    = VT_t*sin(PSI_t + ALPHAT_t);
dx_Alt      = subs(dx_Alt_t,symfunVariables,symVariables);
dy_Alt      = subs(dy_Alt_t,symfunVariables,symVariables);

% Acceleration truck - Alternative definition
ddx_Alt_t   = simplify(diff(dx_Alt_t,t));
ddy_Alt_t   = simplify(diff(dy_Alt_t,t));
ddx_Alt     = subs(ddx_Alt_t,symfunVariables,symVariables);
ddy_Alt     = subs(ddy_Alt_t,symfunVariables,symVariables);

%% Slip angles
% <html>
% <p>Using equations \ref{eq:velfront}, \ref{eq:velrear} and \ref{eq:velsemitrailer}, the slip angles can be written as</p>
% <p>
% \begin{eqnarray}
% \nonumber \alpha_{\rm F} &=& \arctan \left( \frac{\dot{y} + a \dot{\psi} \cos \psi}{ \dot{x} - a \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm F} + \psi \right) \\
% \nonumber \alpha_{\rm R} &=& \arctan \left( \frac{\dot{y} - b \dot{\psi} \cos \psi}{ \dot{x} + b \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm R} + \psi \right)  \\
% \alpha_{\rm M} &=& \arctan \left( \frac{ \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) }{ \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) } \right) - \left( \delta_{\rm M} + \psi - \phi \right) \label{eq:slipangles1}
% \end{eqnarray}
% </p>
% <p>Using equations in \ref{eq:dxdy2vtalphat}, the slip angles become</p>
% <p>
% \begin{eqnarray}
% \alpha_{\rm F} &=& \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) + a \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) - a \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm F} + \psi \right) \label{eq:slipangleFfull} \\
% \alpha_{\rm R} &=& \arctan \left( \frac{v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) - b \dot{\psi} \cos \psi}{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) + b \dot{\psi} \sin \psi} \right) - \left( \delta_{\rm R} + \psi \right) \label{eq:slipangleRfull} \\
% \alpha_{\rm M} &=& \arctan \left( \frac{ v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) - \left( b + c \right) \dot{\psi} \cos \psi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) }{ v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) + \left( b + c \right) \dot{\psi} \sin \psi + \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) } \right) - \left( \delta_{\rm M} + \psi - \phi \right) \label{eq:slipangleMfull}
% \end{eqnarray}
% </p>
% <p>Since the definition of slip angle does not actually depend on the orientation, the orientation may be considered zero for simplification. For the tractor, orientation is \(\psi\). Thus, (\(\psi=0\)) in \ref{eq:slipangleFfull} and \ref{eq:slipangleRfull}. For the semitrailer, orientation is \(\psi-\phi\). Thus, \(\psi-\phi=0 \rightarrow  \psi=\phi\) in \ref{eq:slipangleMfull}. Then </p>
% <p>
% \begin{eqnarray}
% \nonumber \alpha_{\rm F} &=& \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} + a \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta_{\rm F} \\
% \alpha_{\rm R} &=& \arctan \left( \frac{v_{\rm T} \sin \alpha_{\rm T} - b \dot{\psi}}{ v_{\rm T} \cos \alpha_{\rm T}} \right) - \delta_{\rm R}  \\
% \alpha_{\rm M} &=& \arctan \left( \frac{ v_{\rm T} \sin \left( \phi + \alpha_{\rm T} \right) - \left( b + c \right) \dot{\psi} \cos \phi - \left( d + e \right) \left( \dot{\psi} - \dot{\phi} \right) }{ v_{\rm T} \cos \left( \phi + \alpha_{\rm T} \right) + \left( b + c \right) \dot{\psi} \sin \phi } \right) - \delta_{\rm M}
% \label{eq:slipanglesnewvariables}
% \end{eqnarray}
% </p>
% </html>

VF_t    = diff(PFO_t,t);
VR_t    = diff(PRO_t,t);
VM_t    = diff(PMO_t,t);

VF      = formula(subs(VF_t,symfunVariables,symVariables));
VR      = formula(subs(VR_t,symfunVariables,symVariables));
VM      = formula(subs(VM_t,symfunVariables,symVariables));

VF_alt  = simplify(subs(VF,[dx dy],[dx_Alt dy_Alt]));
VR_alt  = simplify(subs(VR,[dx dy],[dx_Alt dy_Alt]));
VM_alt  = simplify(subs(VM,[dx dy],[dx_Alt dy_Alt]));

% Slip angle definition
ALPHAF_def = atan2(VF_alt(2),VF_alt(1)) - (deltaf + PSI);
ALPHAR_def = atan2(VR_alt(2),VR_alt(1)) - (deltar + PSI);
ALPHAM_def = atan2(VM_alt(2),VM_alt(1)) - (deltam + PSI-PHI);

% Slip angle simplification (PSI=0 - Orientation is irrelevant here)
% Orientation can be neglected, i.e., for the truck: Orientation=PSI=0 
ALPHAF = subs(ALPHAF_def,PSI,0);
ALPHAR = subs(ALPHAR_def,PSI,0);
% and for the semitrailer: Orientation=PSI-PHI=0 ---> PSI=PHI
ALPHAM = subs(ALPHAM_def,PSI,PHI);

%%
disp(ALPHAF)
%%
disp(ALPHAR)
%%
disp(ALPHAM) 

%% Forces
% <html>
% <p>Free body diagram</p>
% <p><img vspace="5" hspace="5" src="modelArticulatedFreeBodyDiagram.svg" alt=""> </p>
% <p>Force vectors at front, rear and semitrailer axles is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber {\bf F}_{\rm F} &=& F_{x,{\rm F}} \, {\bf e}_x + F_{y,{\rm F}} \, {\bf e}_y \\
%     \nonumber {\bf F}_{\rm R} &=& F_{x,{\rm R}} {\bf t}_x + F_{y,{\rm R}} {\bf t}_y \\
%     \nonumber {\bf F}_{\rm M} &=& F_{x,{\rm M}} {\bf s}_x + F_{y,{\rm M}} {\bf s}_y
%     \end{eqnarray}
% </p>
% <p>or</p>
% <p>
%     \begin{eqnarray}
%     {\bf F}_{\rm F} &=& \left[ F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) \right] {\bf i} + \left[ F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) \right] {\bf j} \label{eq:forcefront} \\
%     {\bf F}_{\rm R} &=& \left[ F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) \right] {\bf i} + \left[ F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) \right] {\bf j}. \label{eq:forcerear} \\
%     {\bf F}_{\rm M} &=& \left[ F_{x,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) - F_{y,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) \right] {\bf i} + \left[ F_{x,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) + F_{y,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) \right] {\bf j}. \label{eq:forcesemitrailer}
%     \end{eqnarray}
% </p>
% </html>

% Force vectors
FF = [FxF*cos(PSI+deltaf)-FyF*sin(PSI+deltaf)           FxF*sin(PSI+deltaf)+FyF*cos(PSI+deltaf)     0];
FR = [FxR*cos(PSI+deltar)-FyR*sin(PSI+deltar)           FxR*sin(PSI+deltar)+FyR*cos(PSI+deltar)     0];
FM = [FxM*cos(PSI-PHI+deltam)-FyM*sin(PSI-PHI+deltam)   FxM*sin(PSI-PHI+deltam)+FyM*cos(PSI-PHI+deltam)    0];

%%
% <html>
% <p>The generalized forces are</p>
% <p>
%     \begin{equation}
%     Q_k = \sum_{j = 1} ^p {\bf F}_j \cdot \frac{\partial {\bf p}_j}{\partial q_k} \qquad \qquad \begin{array}{c} k = 1, 2, 3, 4 \\ j = {\rm F}, {\rm R}, {\rm M} \end{array}.
%     \end{equation}
% </p>
% <p>This is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber Q_1 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_1} \\
%     \nonumber Q_2 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_2} \\
%     \nonumber Q_3 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_3} \\
%     Q_4 &=& {\bf F}_{\rm F} \cdot \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_4} + {\bf F}_{\rm R} \cdot \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_4} + {\bf F}_{\rm M} \cdot \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_4}.\end{eqnarray}
% </p>
% <p>Thus</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_1} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial x} = {\bf i} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_2} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial y} = {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_3} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \psi} = - a \sin \psi {\bf i} + a \cos \psi {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial q_4} &=& \frac{\partial {\bf p}_{{\rm F}/{\rm O}}}{\partial \phi} = 0,
%     \end{eqnarray}
% </p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_1} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial x} = {\bf i} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_2} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial y} = {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_3} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \psi} = b \sin \psi {\bf i} - b \cos \psi {\bf j} \\
%     \nonumber \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial q_4} &=& \frac{\partial {\bf p}_{{\rm R}/{\rm O}}}{\partial \phi} = 0
%     \end{eqnarray}
% </p>
% <p> and</p>
% <p>
%     \begin{eqnarray}
%     \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_1} &=& \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial x} = {\bf i} \\
%     \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_2} &=& \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial y} = {\bf j} \\
%     \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_3} &=& \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial \psi} = \left[ \left( b + c \right) \sin \psi + \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ - \left( b + c \right) \cos \psi - \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf j} \\
%     \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial q_4} &=& \frac{\partial {\bf p}_{{\rm M}/{\rm O}}}{\partial \phi} = \left[ - \left( d + e \right) \sin \left( \psi - \phi \right) \right] {\bf i} + \left[ \left( d + e \right) \cos \left( \psi - \phi \right) \right] {\bf j}
%     \end{eqnarray}
% </p>
% </html>

% Generalized forces
Q1 = FF*diff(PFO,x)'    + FR*diff(PRO,x)'    + FM*diff(PMO,x)';
Q2 = FF*diff(PFO,y)'    + FR*diff(PRO,y)'    + FM*diff(PMO,y)';
Q3 = FF*diff(PFO,PSI).' + FR*diff(PRO,PSI).' + FM*diff(PMO,PSI).';
Q4 = FF*diff(PFO,PHI).' + FR*diff(PRO,PHI).' + FM*diff(PMO,PHI).';

%%
% <html>
% <p>Substituting equations</p>
% <p>
%     \begin{eqnarray}
%     \nonumber Q_1 &=& F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{x,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) - F_{y,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) \\
%     \nonumber Q_2 &=& F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) + F_{x,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) + F_{y,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) \\
%     \nonumber Q_3 &=& F_{x,{\rm F}} a \sin(\delta_{\rm F}) + F_{y,{\rm F}} a \cos(\delta_{\rm F}) - F_{x,{\rm R}} b \sin(\delta_{\rm R}) - F_{y,{\rm R}} b \cos(\delta_{\rm R}) + F_{x,{\rm M}} \left[ b \sin(\phi - \delta_{\rm M}) - e \sin(\delta_{\rm M}) - d \sin(\delta_{\rm M}) + c \sin(\phi - \delta_{\rm M}) \right] + F_{y,{\rm M}} \left[ - d \cos(\delta_{\rm M}) - e \cos(\delta_{\rm M}) - b \cos(\phi - \delta_{\rm M}) - c \cos(\phi - \delta_{\rm M}) \right] \\
%     \nonumber Q_4 &=& F_{x,{\rm M}} \left( d + e \right) \sin(\delta_{\rm M}) + F_{y,{\rm M}} \left( d + e \right) \cos(\delta_{\rm M})
%     \end{eqnarray}
% </p>
% </html>

% Generalized forces array
Q = formula(simplify([Q1 ; Q2 ; Q3 ; Q4]));

%%
disp(Q)

%% Kinetic Energy
% <html>
% <p>The kinetic energy of the system is</p>
% <p>
%     \begin{equation} \label{eq:kineticenergy} T = \frac{1}{2} m_{\rm T} {\bf v}_{\rm T} \cdot {\bf v}_{\rm T} + \frac{1}{2} m_{S} {\bf v}_{\rm S} \cdot {\bf v}_{\rm S} + \frac{1}{2} \left\{ {\bf w}_{\rm T} \right\}^T \left[ {\bf J}_{\rm T} \right] \left\{ {\bf w}_{\rm T} \right\} + \frac{1}{2} \left\{ {\bf w}_{\rm S} \right\}^T \left[ {\bf J}_{\rm S} \right] \left\{ {\bf w}_{\rm S} \right\}.\end{equation}
% </p>
% <p>Substituting \ref{eq:tractorvelangular}, \ref{eq:semitrailervelangular}, \ref{eq:tractorveltranslation} and \ref{eq:semitrailerveltranslation} in \ref{eq:kineticenergy} </p>
% <p>
%     \begin{equation}T = \frac{1}{2} m_{\rm T} \left( \dot{x}^2 + \dot{y}^2 \right) + \frac{1}{2} m_{\rm S} \left( C_1^2 + C_2^2 \right) + \frac{1}{2} I_{\rm T} \dot{\psi}^2 + \frac{1}{2} I_{\rm S} \left( \dot{\psi} - \dot{\phi} \right)^2,\end{equation}
% </p>
% <p>where</p>
% <p>
%     \begin{eqnarray}
%     \nonumber C_1 &=& \dot{x} + \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \\
%     C_2 &=& \dot{y} - \left( b + c \right) \dot{\psi} \cos \psi - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right). \label{eq:equationconstants}
%     \end{eqnarray}
% </p>
% <p>The time derivatives of \ref{eq:equationconstants} is</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \dot{C}_1 &=& \ddot{x} + \left( b + c \right) \ddot{\psi} \sin \psi + \left( b + c \right) \dot{\psi}^2 \cos \psi + d \left( \ddot{\psi} - \ddot{\phi} \right) \sin \left( \psi - \phi \right) + d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) \\
%     \dot{C}_2 &=& \ddot{y} - \left( b + c \right) \ddot{\psi} \cos \psi + \left( b + c \right) \dot{\psi}^2 \sin \psi - d \left( \ddot{\psi} - \ddot{\phi} \right) \cos \left( \psi - \phi \right) + d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right)
%     \end{eqnarray}
% </p>
% </html>

T = 1/2*mT*(VTdiff*VTdiff.') + 1/2*mS*(vSdiff*vSdiff.') + 1/2*IT*dPSI^2 + 1/2*IS*(dPSI-dPHI)^2;

%% Euler-Lagrange
% <html>
% <p>The Euler-Lagrange formulation for this system is</p>
% <p>
%     \begin{equation}
%     \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_k} \right) - \frac{\partial T}{\partial q_k} = Q_k \qquad \qquad k = 1, 2, 3, 4
%     \end{equation}
% </p>
% <p>where</p>
% <p>
%     \begin{eqnarray}
%     \frac{\partial T}{\partial q_1} &=& \frac{\partial T}{\partial x} = 0 \\
%     \frac{\partial T}{\partial q_2} &=& \frac{\partial T}{\partial y} = 0 \\
%     \frac{\partial T}{\partial q_3} &=& \frac{\partial T}{\partial \psi} = m_S C_1 \left[ \left( b + c \right) \dot{\psi} \cos \psi + d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] \\
%     \frac{\partial T}{\partial q_4} &=& \frac{\partial T}{\partial \phi} = m_S C_1 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right].
%     \end{eqnarray}
% </p>
% </html>

dTdq1 = diff(T,x);
dTdq2 = diff(T,y);
dTdq3 = diff(T,PSI);
dTdq4 = diff(T,PHI);
dTdq = [ dTdq1 ; dTdq2 ; dTdq3 ; dTdq4 ];
dTdq = subs(dTdq,symVariables,symfunVariables);

%%
% <html>
% <p>The partial derivatives are</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{\partial T}{\partial \dot{q}_1} &=& \frac{\partial T}{\partial \dot{x}} = m_{\rm T} \dot{x} + m_S C_1 \\
%     \nonumber \frac{\partial T}{\partial \dot{q}_2} &=& \frac{\partial T}{\partial \dot{y}} = m_{\rm T} \dot{y} + m_S C_2 \\
%     \nonumber \frac{\partial T}{\partial q_3} &=& \frac{\partial T}{\partial \dot{\psi}} = m_S C_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + m_S C_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + I_T \dot{\psi} + I_S \left( \dot{\psi} - \dot{\phi} \right) \\
%     \frac{\partial T}{\partial q_4} &=& \frac{\partial T}{\partial \dot{\phi}} = m_S C_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S C_2 \left[ d \cos \left( \psi - \phi \right) \right] - I_S \left( \dot{\psi} - \dot{\phi} \right).
%     \end{eqnarray}
% </p>
% <p>Time derivatives</p>
% <p>
%     \begin{eqnarray}
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_1} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{x}} \right) = m_{\rm T} \ddot{x} + m_S \dot{C}_1 \\
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_2} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{y}} \right) = m_{\rm T} \ddot{y} + m_S \dot{C}_2 \\
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_3} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{\psi}} \right) = m_S \dot{C}_1 \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] + m_S C_1 \left[ \left( b + c \right) \dot{\psi} \cos \psi + d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ - \left( b + c \right) \cos \psi - d \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ \left( b + c \right) \dot{\psi} \sin \psi + d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] + I_T \ddot{\psi} + I_S \left( \ddot{\psi} - \ddot{\phi} \right) \\
%     \nonumber \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{q}_4} \right) &=& \frac{d}{dt} \left( \frac{\partial T}{\partial \dot{\phi}} \right) = m_S \dot{C}_1 \left[ - d \sin \left( \psi - \phi \right) \right] + m_S C_1 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \cos \left( \psi - \phi \right) \right] + m_S \dot{C}_2 \left[ d \cos \left( \psi - \phi \right) \right] + m_S C_2 \left[ - d \left( \dot{\psi} - \dot{\phi} \right) \sin \left( \psi - \phi \right) \right] - I_S \left( \ddot{\psi} - \ddot{\phi} \right)
%     \end{eqnarray}
% </p>
% </html>

dTddq1 = diff(T,dx);
dTddq2 = diff(T,dy);
dTddq3 = diff(T,dPSI);
dTddq4 = diff(T,dPHI);
dTddq  = [ dTddq1 ; dTddq2 ; dTddq3 ; dTddq4 ];
dTddq  = subs(dTddq,symVariables,symfunVariables);
ddTddq = diff(dTddq,t);

%% State equations
% <html>
% <p>The state variables are</p>
% <p>
%     \begin{eqnarray}
%     {\rm x}_1 &=& x               \\
%     {\rm x}_2 &=& y               \\
%     {\rm x}_3 &=& \psi            \\
%     {\rm x}_4 &=& \phi            \\
%     {\rm x}_5 &=& v_{\rm T}       \\
%     {\rm x}_6 &=& \alpha_{\rm T}  \\
%     {\rm x}_7 &=& \dot{\psi}      \\
%     {\rm x}_8 &=& \dot{\phi}
%     \end{eqnarray}
% </p>
% <p>State space equation</p>
% <p>
%     \begin{equation}
%     {\bf M} \left( {\bf x} \right) \dot{{\bf x}} = {\bf f} \left( {\bf x}, {\bf u} \right),
%     \end{equation}
% </p>
% <p>state vector</p>
% <p>
%     \begin{eqnarray}
%     {\bf x} = \left[ \begin{array}{c} {\rm x}_{1} \\ {\rm x}_{2} \\ {\rm x}_{3} \\ {\rm x}_{4} \\ {\rm x}_{5} \\ {\rm x}_{6} \\ {\rm x}_{7} \\ {\rm x}_{8} \end{array} \right]
%     \end{eqnarray}
% </p>
% <p>input vector is</p>
% <p>
%     \begin{eqnarray}
%     {\bf u} = \left[ \begin{array}{c} \delta_{\rm F} \\ \delta_{\rm R} \\ \delta_{\rm M} \\ F_{x,{\rm F}} \\ F_{x,{\rm R}} \\ F_{x,{\rm M}} \\ F_{y,{\rm F}} \\ F_{y,{\rm R}} \\ F_{y,{\rm M}} \end{array} \right].
%     \end{eqnarray}
% </p>
% <p>Mass matrix is</p>
% <p>
%     \begin{eqnarray}
%     {\bf M} = \left[ \begin{array}{ccccccccc} 1 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 \\ 0 &amp; 1 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 \\0 &amp; 0 &amp; 1 &amp; 0 &amp; 0 &amp; 0 &amp; 0 &amp; 0 \\ 0 &amp; 0 &amp; 0 &amp; 1 &amp; 0 &amp; 0 &amp; 0 &amp; 0 \\ 0 &amp; 0 &amp; 0 &amp; 0 &amp; M_{55} &amp; M_{56} &amp; M_{57} &amp; M_{58} \\ 0 &amp; 0 &amp; 0 &amp; 0 &amp; M_{65} &amp; M_{66} &amp; M_{67} &amp; M_{68} \\ 0 &amp; 0 &amp; 0 &amp; 0 &amp; M_{75} &amp; M_{76} &amp; M_{77} &amp; M_{78} \\ 0 &amp; 0 &amp; 0 &amp; 0 &amp; M_{85} &amp; M_{86} &amp; M_{87} &amp; M_{88} \end{array} \right],
%     \end{eqnarray}
% </p>
% <p>where</p>
% <p>
%     \begin{eqnarray}
%     \nonumber M_{55} &=& \left( m_{\rm T} + m_{\rm S} \right) \cos \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber M_{56} &=& - \left( m_{\rm T} + m_{\rm S} \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber M_{57} &=& m_{\rm S} \left[ \left( b + c \right) \sin \psi + d \sin \left( \psi - \phi \right) \right] \\
%     \nonumber M_{58} &=& - m_{\rm S} d \sin \left( \psi - \phi \right) \\
%     \nonumber M_{65} &=& \left( m_{\rm T} + m_{\rm S} \right) \sin \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber M_{66} &=& \left( m_{\rm T} + m_{\rm S} \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \\
%     \nonumber M_{67} &=& - m_{\rm S} \left[ \left( b + c \right) \cos \psi + d \cos \left( \psi - \phi \right) \right] \\
%     \nonumber M_{68} &=& m_{\rm S} d \cos \left( \psi - \phi \right) \\
%     \nonumber M_{75} &=& - m_{\rm S} \left[ \left( b + c \right) \sin \alpha_{\rm T} + d \sin \left( \alpha_{\rm T} + \phi \right) \right] \\
%     \nonumber M_{76} &=& - m_{\rm S} \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \\
%     \nonumber M_{77} &=& m_{\rm S} \left[ \left( b + c \right)^2 + 2 \left( b + c \right) d \cos \phi + d^2 \right] + I_T + I_{\rm S} \\
%     \nonumber M_{78} &=& - m_{\rm S} \left[ \left( b + c \right) d \cos \phi + d^2 \right] - I_{\rm S} \\
%     \nonumber M_{85} &=& m_{\rm S} d \sin \left( \alpha_{\rm T} + \phi \right) \\
%     \nonumber M_{86} &=& m_{\rm S} d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \\
%     \nonumber M_{87} &=& - m_{\rm S} \left[ \left( b + c \right) d \cos \phi + d^2 \right] - I_{\rm S} \\
%     \nonumber M_{88} &=& m_{\rm S} d^2 + I_{\rm S} 
%     \end{eqnarray}
% </p>
% </html>

% Euler-Lagrante - Left side
EQ = ddTddq-dTdq;
EQ = subs(EQ,symfunVariables,symVariables);
EQ = formula(simplify(subs(EQ,[dx dy ddx ddy],[dx_Alt dy_Alt ddx_Alt ddy_Alt])));

dvar = [dVT dALPHAT ddPSI ddPHI];

% Prealocating
M = sym(eye(8));
for i=1:length(EQ)
    for j=1:length(dvar)
        [C,~]       = coeffs(EQ(i),dvar(j));
        M(i+4,j+4)    = C(1);
    end
end

M = simplify(M);
disp(M)

%%
% <html>
% <p>The \({\bf f}\) function is</p>
% <p>
%     \begin{equation*}
%     {\bf f} = \left[ \begin{array}{c} v_{\rm T} \cos \left(\psi + \alpha_{\rm T} \right) \\ v_{\rm T} \sin \left(\psi + \alpha_{\rm T} \right) \\ \dot{\psi} \\ \dot{\phi} \\ f_5 \\ f_6 \\ f_7 \\ f_8 \\  \end{array} \right],
%     \end{equation*}
% </p>
% <p>where</p>
% <p>
%     \begin{eqnarray}
%         f_{5} &=& F_{x,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) - F_{y,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) - F_{y,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{x,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) - F_{y,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) - m_S \left( b + c \right) \dot{\psi}^2 \cos \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \cos \left( \psi - \phi \right) + \left( m_{\rm T} + m_S \right) v_{\rm T} \sin \left( \psi + \alpha_{\rm T} \right) \dot{\psi} \\
%         f_{6} &=& F_{x,{\rm F}} \sin \left( \psi + \delta_{\rm F} \right) + F_{y,{\rm F}} \cos \left( \psi + \delta_{\rm F} \right) + F_{x,{\rm R}} \sin \left( \psi + \delta_{\rm R} \right) + F_{y,{\rm R}} \cos \left( \psi + \delta_{\rm R} \right) + F_{x,{\rm M}} \sin \left( \psi - \phi + \delta_{\rm M} \right) + F_{y,{\rm M}} \cos \left( \psi - \phi + \delta_{\rm M} \right) - m_S \left( b + c \right) \dot{\psi}^2 \sin \psi - m_S d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \left( \psi - \phi \right) - \left( m_{\rm T} + m_S \right) v_{\rm T} \cos \left( \psi + \alpha_{\rm T} \right) \dot{\psi} \\
%         f_{7} &=& F_{x,{\rm F}} a \sin(\delta_{\rm F}) + F_{y,{\rm F}} a \cos(\delta_{\rm F}) - F_{x,{\rm R}} b \sin(\delta_{\rm R}) - F_{y,{\rm R}} b \cos(\delta_{\rm R}) + F_{x,{\rm M}} \left[ b \sin(\phi - \delta_{\rm M}) - e \sin(\delta_{\rm M}) - d \sin(\delta_{\rm M}) + c \sin(\phi - \delta_{\rm M}) \right] + F_{y,{\rm M}} \left[ - d \cos(\delta_{\rm M}) - e \cos(\delta_{\rm M}) - b \cos(\phi - \delta_{\rm M}) - c \cos(\phi - \delta_{\rm M}) \right] - m_S \left( b + c \right) d \left( \dot{\psi} - \dot{\phi} \right)^2 \sin \phi + m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi + m_S \left[ \left( b + c \right) v_{\rm T} \cos \alpha_{\rm T} + d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \right] \dot{\psi} \\
%         f_{8} &=& F_{x,{\rm M}} \left( d + e \right) \sin(\delta_{\rm M}) + F_{y,{\rm M}} \left( d + e \right) \cos(\delta_{\rm M}) - m_S \left( b + c \right) d \dot{\psi}^2 \sin \phi - m_S d v_{\rm T} \cos \left( \alpha_{\rm T} + \phi \right) \dot{\psi}.
%     \end{eqnarray}
% </p>
% </html>

% Euler-Lagrante - Right side

dq_Alt = formula(subs(dq,[dx dy ddx ddy],[dx_Alt dy_Alt ddx_Alt ddy_Alt]));

f1 = dq_Alt.';
f2 = simplify(Q - (EQ - M(5:end,5:end)*dvar.'));
f = [ f1 ; f2];
disp(f)

%% Model functions
% This section creates matlab functions with the different parts of the
% model.

matlabFunction(f,'File','f_function','Comments','Version: 1.1')
matlabFunction(M,'File','M_function','Comments','Version: 1.1')

%% See Also
%
% <../../../index.html Home>
%