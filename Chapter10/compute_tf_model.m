function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% Lateral coeffiennts
a_phi1 = -((1/2)*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_p)*(P.b/(2*P.Va0));
a_phi2 = (1/2)*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_delta_a;

beta_star = asin(P.v0/sqrt(P.u0^2+P.v0^2+P.w0^2));
a_beta1 = -((P.rho*P.Va0*P.S_wing)/(2*P.mass)) * P.C_Y_beta;
a_beta2 = ((P.rho*P.Va0*P.S_wing)/(2*P.mass)) * P.C_Y_delta_r; 

% Longitudinal coeffiennts
a_theta1 = -(((P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_q) * (P.c/(2*P.Va0)));
a_theta2 = -(((P.rho*P.Va0^2 * P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_alpha));
a_theta3 = ((P.rho*P.Va0^2 * P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_delta_e);

alpha_star = atan(P.w0/P.u0);
a_V1 = ((P.rho*P.Va0*P.S_wing/P.mass) * (P.C_D_0+P.C_D_alpha*alpha_star+P.C_D_delta_e*u_trim(1))) + ...
       (P.rho*P.S_prop/P.mass)*P.C_prop*P.Va0;
a_V2 = ((P.rho*P.S_prop)/P.mass)*P.C_prop*P.k_motor^2*u_trim(4);
a_V3 = P.gravity;


% Lateral TF

% [delta_a -> phi]
% ignore disturbance d_phi2 (too small)
T_phi_delta_a   = tf([a_phi2], [1,a_phi1,0]);

% [phi -> chi]
% ignore disturbance d_chi (too small)
% V_g = V_a because of no wind speed assumption
T_chi_phi       = tf([P.gravity/sqrt(P.u0^2+P.v0^2+P.w0^2)], [1,0]);

% [delta_r -> beta -> v]
% ignore disturbance d_beta (too small)
T_v_delta_r     = tf([a_beta2 * P.Va0], [cos(beta_star),a_beta1]);

% Longitudinal TF

% [delta_e -> theta]
% ignore disturbance d_theta2 (too small)
T_theta_delta_e = tf([a_theta3], [1,a_theta1,a_theta2]);

% [theta -> h]
% ignore disturbance d_theta2 (too small)
T_h_theta       = tf([sqrt(P.u0^2+P.v0^2+P.w0^2)], [1,0]);

% [V_a -> h]
% ignore disturbance d_h (too small)
% (V_a*theta)/s = h
T_h_Va          = tf([P.theta0], [1,0]);

% [delta_t -> V_a]
% ignore disturbance d_V1 (too small)
T_Va_delta_t    = tf([a_V2], [1,a_V1]);

% [theta -> V_a]
% ignore disturbance d_V2 (too small)
T_Va_theta      = tf([-a_V3], [1,a_V1]);

