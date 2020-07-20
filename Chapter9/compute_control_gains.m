%% Roll attitude hold loop example

% Designed parameter
delta_a_max = 45*pi/180;
e_phi_max = 60*pi/180*2;
zeta_phi = 2;

a_phi1 = -((1/2)*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_p)*(P.b/(2*P.Va0));
a_phi2 = (1/2)*P.rho*P.Va0^2*P.S_wing*P.b*P.C_p_delta_a;

P.k_p_phi = (delta_a_max/e_phi_max)*sign(a_phi2);
w_n_phi = sqrt(P.k_p_phi*a_phi2);
P.k_d_phi = (2*zeta_phi*w_n_phi-a_phi1)/a_phi2;

% (Wx = 20; Wh = 15; Wv2 = 10;)
%% Course hold Loop 

% Designed parameter
zeta_chi = 1.2;
W_chi = 20;

% phi -> chi
[num,den] = tfdata(T_chi_phi, 'v');
% num(2) == g/V_g

w_n_chi = (1/W_chi)*w_n_phi;
P.k_p_chi = 2*zeta_chi*w_n_chi/num(2);
P.k_i_chi = (w_n_chi^2)/num(2);
 
%% Sideslip hold Loop 

% Designed parameter
zeta_beta = 1.2;
e_beta_max  = 3*pi/180;
delta_r_max = 20*pi/180;

% TF: T_v_delta_r
a_beta1 = -((P.rho*P.Va0*P.S_wing)/(2*P.mass)) * P.C_Y_beta;
a_beta2 = ((P.rho*P.Va0*P.S_wing)/(2*P.mass)) * P.C_Y_delta_r; 

P.k_p_beta = delta_r_max/e_beta_max;
w_n_beta = (a_beta1+a_beta2*P.k_p_beta)/(2*zeta_beta);
P.k_i_beta = w_n_beta^2/a_beta2;

%% Pitch attitude hold loop 

% Designed parameter
e_theta_max = 20*pi/180;
delta_e_max = 45*pi/180;
zeta_theta = 0.9;

% elevator -> theta
a_theta1 = -(((P.rho*P.Va0^2*P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_q) * (P.c/(2*P.Va0)));
a_theta2 = -(((P.rho*P.Va0^2 * P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_alpha));
a_theta3 = ((P.rho*P.Va0^2 * P.c*P.S_wing)/(2*P.Jy)) * (P.C_m_delta_e);

P.k_p_theta = (delta_e_max/e_theta_max)*sign(a_theta3);
w_n_theta = sqrt(a_theta2 + P.k_p_theta*a_theta3);
P.k_d_theta = (2*zeta_theta*w_n_theta-a_theta1)/a_theta3;
P.theta_DC = (P.k_p_theta*a_theta3)/(a_theta2 + P.k_p_theta*a_theta3);

%% Altitude from pitch gain calculations

% Designed parameter
zeta_h = 0.8;
Wh = 15;
V_a = sqrt(P.u0^2+P.v0^2+P.w0^2);

w_n_h = (1/Wh)*w_n_theta;
P.k_i_h = w_n_h^2/(P.theta_DC*V_a);
P.k_p_h = (2*zeta_h*w_n_h)/(P.theta_DC*V_a);

%% Airspeed from pitch gain calculations 

% Designed parameter
zeta_V2 = 0.05;
Wv2 = 10;

% delta_t -> V_a
[num,den] = tfdata(T_Va_delta_t, 'v');
a_V1 = den(2);

w_n_v2 = (1/Wv2)*w_n_theta;
P.k_i_v2 = w_n_v2^2/(P.theta_DC*P.gravity);
P.k_p_v2 = (a_V1 - 2*zeta_V2*w_n_v2)/(P.theta_DC*P.gravity);

%% Airspeed hold using throttle 

% Designed parameter
w_n_v = 2;
zeta_v = 0.7;

% delta_t -> V_a
[num,den] = tfdata(T_Va_delta_t, 'v');
a_V1 = den(2);
a_V2 = num(2);

P.k_i_v = w_n_v^2/a_V2;
P.k_p_v = (2*zeta_v*w_n_v-a_V1)/a_V2;
