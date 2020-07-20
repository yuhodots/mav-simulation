function y = autopilot(uu,P)

    NN = 0;
    pn       = uu(1+NN);  % inertial North position
%   pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%   alpha    = uu(5+NN);  % angle of attack
%   beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%   Vg       = uu(13+NN); % ground speed
%   wn       = uu(14+NN); % wind North
%   we       = uu(15+NN); % wind East
%   psi      = uu(16+NN); % heading
%   bx       = uu(17+NN); % x-gyro bias
%   by       = uu(18+NN); % y-gyro bias
%   bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
    % autopilot_version == 1 <- used for tuning
    % autopilot_version == 2 <- standard autopilot defined in book
        
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case1: autopilot_tuning - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 3;
    
    switch mode
        
        % tune the roll loop
        case 1,            
            phi_c = chi_c;  % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0;    % no rudder
                            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
            
        % tune the course loop
        case 2,             
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0;    % no rudder
                            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
            
        % tune the throttle to airspeed loop and pitch loop simultaneously    
        case 3, 
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0;    % no rudder
                            % use trim values for elevator and throttle while tuning the lateral autopilot
                            
        % tune the pitch to airspeed loop 
        case 4,
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0;    % no rudder
                            % use trim values for elevator and throttle while tuning the lateral autopilot
        
        % tune the pitch to altitude loop 
        case 5, 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0;    % no rudder
                            % use trim values for elevator and throttle while tuning the lateral autopilot
    end
      
    %---------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% case2: autopilot_uavbook - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = P.climb_out_trottle;
            theta_c = P.theta_c_max;
            if h>=P.altitude_take_off_zone,
                altitude_state = 2;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
        case 2,  % climb zone
            
            delta_t = 1;
            theta_c = -airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if h>=h_c-P.altitude_hold_zone,
                altitude_state = 4;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end
             
        case 3, % descend zone
        
        	delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, initialize_integrator, P);
            if initialize_integrator == 1,
                initialize_integrator = 0;
            end
            if initialize_integrator == 0,
                if h < h_c+P.altitude_hold_zone,
                    altitude_state = 4;
                    initialize_integrator = 1;
                end
            end

        case 4, % altitude hold zone
            
            delta_t = airspeed_with_throttle_hold(Va_c, Va, initialize_integrator, P);
            theta_c = altitude_hold(h_c, h, initialize_integrator, P);
            if h>=h_c+P.altitude_hold_zone,
                altitude_state = 3;
                initialize_integrator = 1;
            else
                initialize_integrator = 0;
            end

    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function delta_a = roll_hold(phi_c, phi, p, P)

  up = P.k_p_phi * (phi_c - phi);
  ud = -P.k_d_phi * p;
  delta_a = sat(up + ud, 45*pi/180, -45*pi/180);
  
end

function phi_c  = course_hold(chi_c, chi, r, first, P)

    persistent sum_course; persistent error_course;
    
    if first==1
        sum_course = 0; error_course = 0;
    end
    
    sum_course = sum_course + (P.Ts)*(chi_c-chi + error_course);
 
    phi_c_temp = P.k_p_chi*error_course + P.k_i_chi*sum_course;
    phi_c = sat(phi_c_temp, 45*pi/180, -45*pi/180); 
    
    
    if P.k_i_chi ~= 0
        sum_course = sum_course + (P.Ts/P.k_i_chi)*(phi_c-phi_c_temp);
    end
    error_course = chi_c-chi;
    
end

function delta_e = pitch_hold(theta_c, theta, q, P)
   
    up = P.k_p_theta*(theta_c - theta);
    ud = -P.k_d_theta * q;
    delta_e = sat(up + ud, 45*pi/180, -45*pi/180);
end

function delta_t = airspeed_with_throttle_hold(Va_c, Va, first, P)

    persistent sum_throttle; persistent error_throttle;
    
    if first==1
        sum_throttle = 0; error_throttle = 0;
    end
    sum_throttle = sum_throttle + (P.Ts)*(error_throttle);

    delta_t_temp = P.k_p_v*error_throttle + P.k_i_v*sum_throttle;
    delta_t = sat(delta_t_temp, 45*pi/180, -45*pi/180);
    
    if P.k_i_v ~= 0
        sum_throttle = sum_throttle + (P.Ts/P.k_i_v)*(delta_t-delta_t_temp);
    end
    error_throttle = Va_c-Va;
end

function theta_c = airspeed_with_pitch_hold(Va_c, Va, first, P)

    persistent sum_pitch; persistent error_pitch;
    
    if first==1
        sum_pitch = 0; error_pitch = 0;
    end
    sum_pitch = sum_pitch + (P.Ts)*(error_pitch);

    theta_c_temp = P.theta_DC* (P.k_p_v2*error_pitch + P.k_i_v2*sum_pitch);
    theta_c = sat(theta_c_temp, 45*pi/180, -45*pi/180);
    
    if P.k_i_h ~= 0
        sum_pitch = sum_pitch + (P.Ts/P.k_i_v2)*(theta_c-theta_c_temp);
    end
    
    error_pitch = Va_c-Va;

end

function theta_c = altitude_hold(h_c, h, first, P)

    persistent sum_altitude; persistent error_altitude;
    
    if first==1
        sum_altitude = 0; error_altitude = 0;
    end
    sum_altitude = sum_altitude + (P.Ts)*(error_altitude);

    theta_c_temp = P.theta_DC*(P.k_p_h*error_altitude + P.k_i_h*sum_altitude);
    theta_c = sat(theta_c_temp, 45*pi/180, -45*pi/180);
    
    if P.k_i_h ~= 0
        sum_altitude = sum_altitude + (P.Ts/P.k_i_h)*(theta_c-theta_c_temp);
    end
    error_altitude = h_c-h;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit
      out = up_limit;
  elseif in < low_limit
      out = low_limit;
      
  else
      out = in;
  end
end
  
 