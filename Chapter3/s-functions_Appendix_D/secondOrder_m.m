function [sys, x0, str, ts] = second_order_m(t, x, u, flag, zeta, wn)

switch flag,
    
  case 0,
    [sys,x0,str,ts] = mdlInitializeSizes;     % initialize block
    
  case 1,
    sys = mdlDerivatives(t,x,u,zeta,wn);      % define xdot = f(t,x,u)
    
  case 3,
    sys = mdlOutputs(t,x,u,wn);               % define xup = g(t,x,u)
    
  otherwise,
      sys = [];
end


% [mdlInitializeSizes]
% Return the sizes, initial conditions, and sample times for the S-function.

function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;       % at least one sample time is needed

sys = simsizes(sizes);

x0  = [0; 0];                   % define initial conditions

str = [];                       % str is always an empty matrix

% initialize the array of sample times

ts  = [0 0];                    % continuous sample time


% [mdlDerivatives]
% Return the derivatives for the continuous states.

function xdot = mdlDerivatives(t,x,u,zeta,wn)

xdot(1) = -2 * zeta * wn * x(1) - wn^2 * x(2) + u;
xdot(2) = x(1);


% [mdlOutputs]
% Return the block outputs.

function sys = mdlOutputs(t,x,u,wn)

sys = wn^2 * x(2);
