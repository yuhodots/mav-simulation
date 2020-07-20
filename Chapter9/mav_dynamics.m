function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  % Initialization
  case 0,
    [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(P);

  % Derivatives
  case 1,
    sys = mdlDerivatives(t,x,u,P);
    
  % Update
  case 2,
    sys = mdlUpdate(t,x,u);

  % Outputs
  case 3,
    sys = mdlOutputs(t,x,u);

  % GetTimeOfNextVarHit 
  case 4,
    sys = mdlGetTimeOfNextVarHit(t,x,u);

  % Terminate
  case 9,
    sys = mdlTerminate(t,x,u);

  % Unexpected flags
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end


% [mdlInitializeSizes]
% Return the sizes, initial conditions, and sample times for the S-function.

function [sys,x0,str,ts,simStateCompliance] = mdlInitializeSizes(P)

% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.

% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.

sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;       % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions

x0  = [...
    P.pn0;...
    P.pe0;...
    P.pd0;...
    P.u0;...
    P.v0;...
    P.w0;...
    P.phi0;...
    P.theta0;...
    P.psi0;...
    P.p0;...
    P.q0;...
    P.r0;...
    ];

% str is always an empty matrix

str = [];

% initialize the array of sample times

ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';


% [mdlDerivatives]
% Return the derivatives for the continuous states.

function sys = mdlDerivatives(t,x,uu, P)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % Equation (3.14)
    pndot = ((cos(theta)*cos(psi)) * u) + ...
            ((sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) * v) + ...
            ((cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) * w);
    pedot = ((cos(theta)*sin(psi)) * u) + ...
            ((sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) * v) + ...
            ((cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) * w);
    pddot = (-sin(theta) * u) + ...
            (sin(phi)*cos(theta) * v) + ...
            (cos(phi)*cos(theta) * w);
    
    % Equation (3.15)
    mass = P.mass;
    
    udot = (r*v - q*w) + (1/mass)*(fx);
    vdot = (p*w - r*u) + (1/mass)*(fy);
    wdot = (q*u - p*v) + (1/mass)*(fz);
    
    % Equation (3.16)
    phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    thetadot = cos(phi)*q - sin(phi)*r; 
    psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
    
    % Equation (3.17)
    Gamma = P.Jx * P.Jz - (P.Jxz)^2;
    Gamma1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/Gamma;
    Gamma2 = (P.Jz*(P.Jz-P.Jy) + (P.Jxz)^2)/Gamma;
    Gamma3 = P.Jz/Gamma;
    Gamma4 = P.Jxz/Gamma;
    Gamma5 = (P.Jz-P.Jx)/P.Jy;
    Gamma6 = P.Jxz/P.Jy;
    Gamma7 = ((P.Jx-P.Jy)*P.Jx + (P.Jxz)^2)/Gamma;
    Gamma8 = P.Jx/Gamma;
    
    pdot = (Gamma1*p*q - Gamma2*q*r) + (Gamma3*ell + Gamma4*n);
    qdot = (Gamma5*p*r - Gamma6*((p^2)-(r^2))) + ((1/P.Jy)*m);
    rdot = (Gamma7*p*q - Gamma1*q*r) + (Gamma4*ell + Gamma8*n);

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];


% [mdlUpdate]
% Handle discrete state updates, sample time hits, and major time step requirements.

function sys = mdlUpdate(t,x,u)

sys = [];


% [mdlOutputs]
% Return the block outputs.

function sys = mdlOutputs(t,x,u)

sys = x;


% [mdlGetTimeOfNextVarHit]
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.

function sys = mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;             % Example, set the next hit to be one second later.
sys = t + sampleTime;


% [mdlTerminate]
% Perform any end of simulation tasks.

function sys = mdlTerminate(t,x,u)

sys = [];
