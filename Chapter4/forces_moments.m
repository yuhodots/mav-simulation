% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % rotation from inertial frame to body frame
    R = [...
        1, 0, 0;...
        0, cos(phi), sin(phi);...
        0, -sin(phi), cos(phi)]*...
        [...
        cos(theta), 0, -sin(theta);...
        0, 1, 0;...
        sin(theta), 0, cos(theta)]*...
        [...
        cos(psi), sin(psi), 0;...
        -sin(psi), cos(psi), 0;...
        0, 0, 1];
    
    % compute wind vector in the body frame
    V_w = R*[w_ns; w_es; w_ds]+[u_wg; v_wg; w_wg];
    u_w = V_w(1);
    v_w = V_w(2);
    w_w = V_w(3);
    
    % compute wind vector in the inertial frame
    temp = (inv(R))*V_w;
    w_n = temp(1); 
    w_e = temp(2); 
    w_d = temp(3);
    
    % compute the velocity relative to the air mass
    u_r      = u - u_w;
    v_r      = v - v_w;
    w_r      = w - w_w;
    
    % compute air data
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    alpha = atan(w_r/u_r);
    beta = asin(v_r/sqrt(u_r^2 + v_r^2 + w_r^2));
    
    % compute external forces and torques on aircraft
    C1 = P.mass * 9.8;
    C2 = (1/2)*P.rho*(Va^2)*P.S_wing;
    C3 = (1/2)*P.rho*P.S_prop*P.C_prop;
    
    % Linear model
    CL = P.C_L_0 + P.C_L_alpha*alpha;
    CD = P.C_D_0 + P.C_D_alpha*alpha;
    
    % Non linear model
    sigma_alpha = (1 + exp(-P.M*(alpha-P.alpha0)) + exp(P.M*(alpha+P.alpha0)))/ ...
                  ((1 + exp(-P.M*(alpha-P.alpha0)))*(1 + exp(P.M*(alpha+P.alpha0))));
    CL = (1-sigma_alpha)*(P.C_L_0 + P.C_L_alpha*alpha) + ...
         sigma_alpha*(2*sign(alpha)*((sin(alpha))^2)*cos(alpha));
    CD = P.C_D_p + ((P.C_L_0 + P.C_L_alpha*alpha)^2)/(pi*P.e*(P.b^2/P.S_wing));
    
    CX = -CD*cos(alpha) + CL*sin(alpha);
    CX_q = -P.C_D_q*cos(alpha) + P.C_L_q*sin(alpha);
    CX_de = -P.C_D_delta_e*cos(alpha) + P.C_L_delta_e*sin(alpha);
    CZ = -CD*sin(alpha) - CL*cos(alpha);
    CZ_q = -P.C_D_q*sin(alpha) - P.C_L_q*cos(alpha);
    CZ_de = -P.C_D_delta_e*sin(alpha) - P.C_L_delta_e*cos(alpha);
    
    Force(1) =  C1*(-sin(theta)) ...
              + C2*(CX + CX_q*(P.c/(2*Va))*q + CX_de*delta_e) ...
              + C3*((P.k_motor*delta_t)^2 - Va^2);
    Force(2) =  C1*(cos(theta)*sin(phi)) ...
              + C2*(P.C_Y_0 + P.C_Y_beta*beta + P.C_Y_p*(P.b/(2*Va))*p + ...
                    P.C_Y_r*(P.b/(2*Va))*r + P.C_Y_delta_a*delta_a + P.C_Y_delta_r*delta_r);
    Force(3) =  C1*(cos(theta)*cos(phi)) ...
              + C2*(CZ + CZ_q*(P.c/(2*Va))*q + CZ_de*delta_e);
    Torque(1) = C2*P.b*(P.C_ell_0 + P.C_ell_beta*beta + P.C_ell_p*(P.b/(2*Va))*p + ...
                        P.C_ell_r*(P.b/(2*Va))*r + P.C_ell_delta_a*delta_a + P.C_ell_delta_r*delta_r) ...
              + (-P.k_T_P*((P.k_Omega*delta_t)^2));
    Torque(2) = C2*P.c*(P.C_m_0 + P.C_m_alpha*alpha + P.C_m_q*(P.c/(2*Va))*q + P.C_m_delta_e*delta_e);
    Torque(3) = C2*P.b*(P.C_n_0 + P.C_n_beta*beta + P.C_n_p*(P.b/(2*Va))*p + ...
                        P.C_n_r*(P.b/(2*Va))*r + P.C_n_delta_a*delta_a + P.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



