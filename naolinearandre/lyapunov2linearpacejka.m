function [Texp,Lexp]=lyapunov2linearpacejka(n,VEICULODADOS,PNEUDADOS,tstart,stept,tend,ystart,ioutp);
%
%    Lyapunov exponent calcullation for ODE-system.
%
%    The alogrithm employed in this m-file for determining Lyapunov
%    exponents was proposed in
%
%         A. Wolf, J. B. Swift, H. L. Swinney, and J. A. Vastano,
%        "Determining Lyapunov Exponents from a Time Series," Physica D,
%        Vol. 16, pp. 285-317, 1985.
%
%    For integrating ODE system can be used any MATLAB ODE-suite methods. 
% This function is a part of MATDS program - toolbox for dynamical system investigation
%    See:    http://www.math.rsu.ru/mexmat/kvm/matds/
%
%    Input parameters:
%      n - number of equation
%      rhs_ext_fcn - handle of function with right hand side of extended ODE-system.
%              This function must include RHS of ODE-system coupled with 
%              variational equation (n items of linearized systems, see Example).                   
%      fcn_integrator - handle of ODE integrator function, for example: @ode45                  
%      tstart - start values of independent value (time t)
%      stept - step on t-variable for Gram-Schmidt renormalization procedure.
%      tend - finish value of time
%      ystart - start point of trajectory of ODE system.
%      ioutp - step of print to MATLAB main window. ioutp==0 - no print, 
%              if ioutp>0 then each ioutp-th point will be print.
%
%    Output parameters:
%      Texp - time values
%      Lexp - Lyapunov exponents to each time value.
%
%    Users have to write their own ODE functions for their specified
%    systems and use handle of this function as rhs_ext_fcn - parameter.      
%
%    Example. Lorenz system:
%               dx/dt = sigma*(y - x)     = f1
%               dy/dt = r*x - y - x*z = f2
%               dz/dt = x*y - b*z     = f3
%
%    The Jacobian of system: 
%        | -sigma  sigma  0 |
%    J = |   r-z    -1   -x |
%        |    y      x   -b |
%
%    Then, the variational equation has a form:
% 
%    F = J*Y
%    where Y is a square matrix with the same dimension as J.
%    Corresponding m-file:
%        function f=lorenz_ext(t,X)
%         SIGMA = 10; R = 28; BETA = 8/3;
%         x=X(1); y=X(2); z=X(3);
%
%         Y= [X(4), X(7), X(10);
%             X(5), X(8), X(11);
%             X(6), X(9), X(12)];
%         f=zeros(9,1);
%         f(1)=SIGMA*(y-x); f(2)=-x*z+R*x-y; f(3)=x*y-BETA*z;
%
%         Jac=[-SIGMA,SIGMA,0; R-z,-1,-x; y, x,-BETA];
%  
%         f(4:12)=Jac*Y;
%
%    Run Lyapunov exponent calculation:
%     
%    [T,Res]=lyapunov(3,@lorenz_ext,@ode45,0,0.5,200,[0 1 0],10);   
%   
%    See files: lorenz_ext, run_lyap.   
%  
% --------------------------------------------------------------------
% Copyright (C) 2004, Govorukhin V.N.
% This file is intended for use with MATLAB and was produced for MATDS-program
% http://www.math.rsu.ru/mexmat/kvm/matds/
% lyapunov.m is free software. lyapunov.m is distributed in the hope that it 
% will be useful, but WITHOUT ANY WARRANTY. 
%

n1=n; % numero de odes nao lineares 
% n1 = 2
n2=n1*(n1+1); % numero total de odes
% n2 = 6 => 2 + 2*2

%  tend=tempo final / stept=step em tempo / tstart=tempo inicial
nit = round((tend-tstart)/stept); % numero de iterações

% Memory allocation - Pre alocacao com zeros de um monte de coisa
y=zeros(n2,1); % condições iniciais comprimento 6 por causa dos dois estados do sistema e os 4 do sistema adicional
cum=zeros(n1,1); % numerador dos expoentes - dois a cada iteração
znorm=cum; % denominador
y0=y; % vetor ?
gsc=cum; % não sei

% Initial values
y(1:n)=ystart(:); % Altera o vetor de condição inicial colocando nas duas primeiras posições a condição inicial do sistema

for i=1:n1 
y((n1+1)*i)=1.0;  % definindo a base ortonormal (bem legal)
end;

t=tstart; % inicializa o tempo

% Main loop

for ITERLYAP=1:nit

% Solutuion of extended ODE system 

%rhs_ext_fcn
  [T,Y] = ode45(@(t,x) linearpacejkalyapunovext(t,x,VEICULODADOS,PNEUDADOS),[t t+stept],y);  

% atualização para a próxima iteração  
  t=t+stept;
  y=Y(size(Y,1),:);	% size(Y,1) da o numero de linhas --> passa apenas a ultima linha para o vetor y (atualizando)

%=========================================================================================================================================
%=========================================================================================================================================
%=========================================================================================================================================
%  for i=1:n1 
%       for j=1:n1 
%		y0(n1*i+j)=y(n1*j+i); % y0 é 0 depois os psi da equação adicional -> y0 fica [0 0 ... [valores]]
%	end;
%  end;

%%       construct new orthonormal basis by gram-schmidt
%  znorm(1)=0.0;
%  for j=1:n1 % se eu nao me engano ele ta fazendo e1=v1/||v1||
%		znorm(1)=znorm(1)+y0(n1*j+1)^2; 
%  end;

%  znorm(1)=sqrt(znorm(1));

%  for j=1:n1 
%		y0(n1*j+1)=y0(n1*j+1)/znorm(1); 
%  end;



%  for j=2:n1
%      for k=1:(j-1)
%          gsc(k)=0.0;
%          for l=1:n1 
%		gsc(k)=gsc(k)+y0(n1*l+j)*y0(n1*l+k); 
%	  end;
%      end;
% 
%      for k=1:n1
%          for l=1:(j-1)
%              y0(n1*k+j)=y0(n1*k+j)-gsc(l)*y0(n1*k+l);
%          end;
%      end;

%      znorm(j)=0.0;
%      for k=1:n1 
%		znorm(j)=znorm(j)+y0(n1*k+j)^2; 
%      end;
%      
%      znorm(j)=sqrt(znorm(j));

%      for k=1:n1 
%		y0(n1*k+j)=y0(n1*k+j)/znorm(j); 
%      end;
%  end;
%=========================================================================================================================================
%=========================================================================================================================================
%=========================================================================================================================================

v1 = y(3:4);
v2 = y(5:6);

znorm(1) = sqrt(v1(1)^2 + v1(2)^2);
e1 = v1/znorm(1);

aux = (v2 - v2*e1'*e1); % variavel auxiliar
znorm(2) = sqrt(aux(1)^2 + aux(2)^2);
e2 = (v2 - v2*e1'*e1)/znorm(2);


y(3:4) = e1;
y(5:6) = e2;

%
%       update running vector magnitudes
%

  for k=1:n1 
	cum(k)=cum(k)+log2(znorm(k)); 
  end;

%
%       normalize exponent
%

  for k=1:n1 
      lp(k)=cum(k)/(t-tstart); 
  end;

% Output modification

  if ITERLYAP==1
     Lexp=lp;
     Texp=t;
  else
     Lexp=[Lexp; lp];
     Texp=[Texp; t];
  end;

  if (mod(ITERLYAP,ioutp)==0)
     fprintf('t=%6.4f',t);
     for k=1:n1 fprintf(' %10.6f',lp(k)); end;
     fprintf('\n');
  end;

%  for i=1:n1 
%      for j=1:n1
%          y(n1*j+i)=y0(n1*i+j);
%      end;
%  end;


end;
