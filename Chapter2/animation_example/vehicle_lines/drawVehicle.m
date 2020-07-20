function drawVehicle(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial Down position
    u        = uu(4);       % body frame velocities
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time

    % define persistent variables 
    persistent vehicle_pts;
    persistent vehicle_handle;
    
    % first time function is called, initialize plot and persistent vars
    if t<=0.1,
        figure(1), clf
        vehicle_pts = defineVehiclePoints;
        vehicle_handle = drawVehicleBody(vehicle_pts,pn,pe,pd,phi,theta,psi, [], 'normal');
        title('Spacecraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-10,10,-10,10,-10,10]);
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawVehicleBody(vehicle_pts,pn,pe,pd,phi,theta,psi, vehicle_handle);
    end
end

  
%=======================================================================
% drawVehicleBody
% return handle if 7th argument is empty, otherwise use 7th arg as handle
%=======================================================================
%
function handle = drawVehicleBody(NED,pn,pe,pd,phi,theta,psi, handle, mode)
  
  NED = rotate(NED,phi,theta,psi); % rotate spacecraft by phi, theta, psi
  NED = translate(NED,pn,pe,pd); % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  
  % plot spacecraft
  if isempty(handle),
    handle = plot3(NED(2,:),NED(1,:),-NED(3,:),'EraseMode', mode);
  else
    set(handle,'XData',NED(2,:),'YData',NED(1,:),'ZData',-NED(3,:));
    drawnow
  end
end
 

%%%%%%%%%%%%%%%%%%%%%%%
function pts=rotate(pts,phi,theta,psi);

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), sin(phi);...
          0, -sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, -sin(theta);...
          0, 1, 0;...
          sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), sin(psi), 0;...
          -sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_roll*R_pitch*R_yaw;  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  R = R';

  % rotate vertices
  pts = R*pts;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function pts = translate(pts,pn,pe,pd)

  pts = pts + repmat([pn;pe;pd],1,size(pts,2));
  
end

% end translateXYZ

%=======================================================================
% defineVehiclePoints
%=======================================================================
function pts = defineVehiclePoints

pts = [...
    1, 0, 0;...   % pt 1
    -1, -2, 0;... % pt 2
    0, 0, 0;...   % pt 3
    -1, 2, 0;...  % pt 4
    1, 0, 0;...   % pt 1
    0, 0, -1;...  % pt 5
    0, 0, 0;...   % pt 3
    1, 0, 0;...   % pt 1
    ]';
end
  