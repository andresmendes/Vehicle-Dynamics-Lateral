function [r,varargout]=circfit(x,y)
%CIRCFIT  Least squares fit of X-Y data to a circle.
%   R = CIRCFIT(X,Y) returns scalar radius R of a fitted circle. X and Y are 1-D
%   arrays of position data in a rectilinear coordinate system. X and Y must be
%   the same length and must contain at least three non-colinear points in order
%   for a valid solution to be found.
%
%   [R,ERR] = CIRCFIT(X,Y) additionally returns the scalar root mean squared
%   error of the fitted circle radius and center relative to the position data.
%
%   [R,XC,YC] = CIRCFIT(X,Y) additionally returns the scalar positions, XC and
%   YC, of center of the fitted circle.
%
%   [R,XC,YC,ERR] = CIRCFIT(X,Y) returns both the center positions of the circle
%   as well as the root mean squared error.
%
%   Examples:
%       % Fit of just five noisy points
%       x1=[1 0 -1 0 1]+0.05*randn(1,5); y1=[0 1 0 -1 0]+0.05*randn(1,5);
%       r1=circfit(x1,y1)
%
%       % CIRCFIT can sometimes perfom poorly with less than 180-degree arc
%       t=0:0.1:pi; lt=length(t);
%       x2=cos(t)+0.04*randn(1,lt); y2=sin(t)+0.04*randn(1,lt);
%       r2_90deg=circfit(x2(1:floor(lt/2)),y2(1:floor(lt/2)))
%       r2_180deg=circfit(x2,y2)

%   Andrew D. Horchler, adh9 @ case . edu, Created 5-12-7
%   Revision: 1.3, 4-6-16


% Check inputs
if nargout > 4
    error('circfit:circfit:TooManyOutputs','Too many output arguments.');
end

if ~isvector(x) || ~isfloat(x) || ~isreal(x) || ~all(isfinite(x))
    error('circfit:circfit:NonFiniteRealVectorX',...
          'X must be a finite real vector of floating point numbers.');
end
if ~isvector(y) || ~isfloat(y) || ~isreal(y) || ~all(isfinite(y))
    error('circfit:circfit:NonFiniteRealVectory',...
          'Y must be a finite real vector of floating point numbers.');
end

lx=length(x);
if lx ~= length(y)
    error('circfit:circfit:LengthMismatch',...
          'The vectors X and Y must have the same length.');
end
if lx < 3
    error('circfit:circfit:Min3Points',...
          'The vectors X and Y must contain at least three points.');
end
x=x(:);
y=y(:);

% Check collinearity, assume with sufficient points, some will be non-collinear
if rank(diff([x([1:min(50,lx) 1]) y([1:min(50,lx) 1])])) == 1
    if lx <= 50 || rank(diff([x y;x(1) y(1)])) == 1
        error('circfit:circfit:Colinearity',...
             ['The points in vectors X and Y must not all be collinear, or '...
              'nearly collinear, with each other.']);
    end
end

xx=x.*x;
yy=y.*y;
xy=x.*y;
xxyy=xx+yy;
sx=sum(x);
sy=sum(y);
sxx=sum(xx);
syy=sum(yy);
sxy=sum(xy);

% Solve linear system without inverting
% a=[sx sy lx;sxy syy sy;sxx sxy sx]\[sxx+syy;sum(xxyy.*y);sum(xxyy.*x)];
[L,U]=lu([sx sy lx;sxy syy sy;sxx sxy sx]);
a=U\(L\[sxx+syy;sum(xxyy.*y);sum(xxyy.*x)]);

xc=0.5*a(1);          	% X-position of center of fitted circle
yc=0.5*a(2);          	% Y-position of center of fitted circle
r=sqrt(xc^2+yc^2+a(3));	% Radius of fitted circle

% Set variable outputs
if nargout > 2
    varargout{1}=xc;
    varargout{2}=yc;
end
if nargout == 2 || nargout == 4
    varargout{nargout-1}=sqrt(mean((sqrt((x-xc).^2+(y-yc).^2)-r).^2));	% RMSE
end
