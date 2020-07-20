
clear all

P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
P.theta_c_max = 30*pi/180; % maximum pitch angle command 
P.climb_out_trottle = 0.5;
P.gravity = 9.8;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params for Aersonade UAV
%physical parameters of airframe
P.mass = 25;
P.Jx   = 0.8244;
P.Jy   = 1.135;
P.Jz   = 1.759;
P.Jxz  = .1204;
% aerodynamic coefficients
P.S_wing        = 0.55;
P.b             = 2.8956;
P.c             = 0.18994;
P.S_prop        = 0.2027;
P.rho           = 1.2682;
P.k_motor       = 80;
P.k_T_P         = 0;
P.k_Omega       = 0;
P.e             = 0.9;

P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.30;
P.C_D_p         = 0.0437;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_m_0         = -0.02338;
P.C_m_alpha     = -0.38;
P.C_m_q         = -3.6;
P.C_m_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = 0.0;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1.0;
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% [Chapter3 parameters]
Gamma  = P.Jx * P.Jz - (P.Jxz)^2;
Gamma1 = P.Jxz*(P.Jx-P.Jy+P.Jz)/Gamma;
Gamma2 = (P.Jz*(P.Jz-P.Jy) + (P.Jxz)^2)/Gamma;
Gamma3 = P.Jz/Gamma;
Gamma4 = P.Jxz/Gamma;
Gamma5 = (P.Jz-P.Jx)/P.Jy;
Gamma6 = P.Jxz/P.Jy;
Gamma7 = ((P.Jx-P.Jy)*P.Jx + (P.Jxz)^2)/Gamma;
Gamma8 = P.Jx/Gamma;

% Additional parameters
P.C_p_0       = (Gamma3*P.C_ell_0) + (Gamma4*P.C_n_0);
P.C_p_beta    = (Gamma3*P.C_ell_beta) + (Gamma4*P.C_n_beta);
P.C_p_p       = (Gamma3*P.C_ell_p) + (Gamma4*P.C_n_p);
P.C_p_r       = (Gamma3*P.C_ell_r) + (Gamma4*P.C_n_p);
P.C_p_delta_a = (Gamma3*P.C_ell_delta_a) + (Gamma4*P.C_n_delta_a);
P.C_p_delta_r = (Gamma3*P.C_ell_delta_r) + (Gamma4*P.C_n_delta_r);

P.C_r_0       = (Gamma4*P.C_ell_0) + (Gamma8*P.C_n_0);
P.C_r_beta    = (Gamma4*P.C_ell_beta) + (Gamma8*P.C_n_beta);
P.C_r_p       = (Gamma4*P.C_ell_p) + (Gamma8*P.C_n_p);
P.C_r_r       = (Gamma4*P.C_ell_r) + (Gamma8*P.C_n_r);
P.C_r_delta_a = (Gamma4*P.C_ell_delta_a) + (Gamma8*P.C_n_delta_a);
P.C_r_delta_r = (Gamma4*P.C_ell_delta_r) + (Gamma8*P.C_n_delta_r);
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% wind parameters
P.wind_n = 3;
P.wind_e = -3;
P.wind_d = 0;
P.L_u = 200;
P.L_v = 200;
P.L_w = 50;
P.sigma_u = 1.06; 
P.sigma_v = 1.06;
P.sigma_w = .7;

% compute trim conditions using 'mavsim_chap5_trim.slx'
% initial airspeed
P.Va0 = 35;
gamma = 0*pi/180;  % desired flight path angle (radians)
R     = Inf;         % desired radius (m) - use (+) for right handed orbit,
h0    = 100;  % initial altitude

% autopilot sample rate
P.Ts = 0.01;

% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Va0; % initial velocity along body x-axis
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% run trim commands
[x_trim, u_trim]=compute_trim('mavsim_trim',P.Va0,gamma,R);
P.u_trim = u_trim;
P.x_trim = x_trim;

% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = -h0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% linearize the equations of motion around trim conditions
[A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,...
 T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r] = compute_tf_model(x_trim,u_trim,P);

%%%
% delta_a_max = 45*pi/180;
% e_phi_max = 60*pi/180*2;
% zeta_roll = 2;
% P.k_p_phi = delta_a_max/e_phi_max*sign(a_phi2);
% w_n_phi = sqrt(abs(a_phi2)*delta_a_max/e_phi_max);
% P.k_d_phi = (2*zeta_roll*w_n_phi-a_phi1)/a_phi2;
% sys_roll_loop = tf([w_n_phi^2],[1 2*zeta_roll*w_n_phi w_n_phi^2]);
% 
% zeta_course = 2;
% w_n_course = w_n_phi/20;
% P.k_p_course = 2*zeta_course*w_n_course*P.Va0/P.gravity;
% P.k_i_course = w_n_course^2*P.Va0/P.gravity;
% sys_course_loop = tf([2*zeta_course*w_n_course w_n_course],[1 2*zeta_course*w_n_course w_n_course^2]);
% 
% delta_r_max = 20*pi/180;
% e_beta_max = 3;
% zeta_beta = 0.707;
% P.k_p_beta = delta_r_max/e_beta_max*sign(a_beta2);
% P.k_i_beta = ((a_beta1+a_beta2*P.k_p_beta)/2/zeta_beta)^2/a_beta2;
% 
% delta_e_max = 45*pi/180;
% e_theta_max = 10*pi/180*2;
% zeta_theta =0.707;
% P.k_p_theta = delta_e_max/e_theta_max*sign(a_theta3);
% w_n_theta = sqrt(a_theta2 + P.k_p_theta*a_theta3);
% P.k_d_theta = (2*zeta_theta*w_n_theta - a_theta1)/a_theta3;
% P.K_theta_DC = P.k_p_theta*a_theta3/(a_theta2+P.k_p_theta*a_theta3);
% 
% w_n_h = w_n_theta/15;
% zeta_h = 1;
% %  P.k_i_h = w_n_h^2/P.K_theta_DC/P.Va0;
% %  P.k_p_h = 2*zeta_h*w_n_h/P.K_theta_DC/P.Va0;
%  P.k_i_h = 0.01;
%  P.k_p_h = 0.1;
% 
% w_n_v2 = w_n_theta/10;
% zeta_v2 = 2;
% P.k_i_v2 = -w_n_v2^2/P.K_theta_DC/P.gravity;
% P.k_p_v2 = (a_V1-2*zeta_v2*w_n_v2)/P.K_theta_DC/P.gravity;
% 
% zeta_v = 2;
% w_n_v = 3;
% P.k_i_v = w_n_v^2/a_V2;
% P.k_p_v = (2*zeta_v*w_n_v-a_V1)/a_V2;


% compute_control_gains
compute_control_gains;
 

% chapter 9 - guidance model
P.gamma_max = 30*pi/180;  % maximum flight path angle
P.phi_max = 45*pi/180;

% wn_chi = 1.5;
% zeta_chi = 0.7;
% P.b_chidot = 2*zeta_chi*wn_chi; 
% P.b_chi = wn_chi^2;
% P.b_Va = 2;
% wn_h = 1;
% zeta_h = 0.4;
% P.b_hdot = 2*wn_h*zeta_h; 
% P.b_hdot_num = 0;
% P.b_h = wn_h^2;

P.b_chidot = 1; P.b_chi = 1;
P.b_h = 2; P.b_hdot = 1;
P.b_Va = 1; P.b_phi = 1;

% chapter 10 - path following
P.chi_infty = 60*pi/180;
P.k_path    = 0.1; P.k_orbit   = 3; P.ki_orbit  = 4; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%







