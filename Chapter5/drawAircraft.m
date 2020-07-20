%% 20161253 Yuho_Jeong

function drawAircraft(uu)

    % Process inputs to function
    
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

    % Define persistent variables 
    
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % First time function is called, 
    % Initialize plot and persistent vars
    
    if t == 0,
        
        figure(1), clf
        
        [Vertices, Faces, facecolors] = defineAircraftBody;
        aircraft_handle = drawBody(Vertices, Faces, facecolors, ...
            pn, pe, pd, phi, theta, psi, ...
            []);
                                     
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32, 47)    % set the view angle for figure
        
        % axis([-10,10,-10,10,-10,10]);
        axis([pe-100, pe+100, pn-100, pn+100, -100, 100]); 

        hold on
        grid on
        
    % At every other time step, redraw aircraft
    
    else 
        
        drawBody(Vertices, Faces, facecolors,...
            pn, pe, pd, phi, theta, psi, aircraft_handle);
                       
        % Move axes with aircraft
        set(aircraft_handle.Parent, 'XLim',[pe-100,pe+100])
        set(aircraft_handle.Parent, 'YLim',[pn-100,pn+100])
       
    end
end


% [drawAircraft]
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle

function handle = drawBody(V, F, patchcolors,...
    pn, pe, pd, phi, theta, psi, handle, mode)

    V = rotate(V, phi, theta, psi);   % rotate vehicle
    V = translate(V, pn, pe, pd);     % translate vehicle
  
    % transform vertices from NED to XYZ (for matlab rendering)
  
    R = [...
            0, 1, 0;...
            1, 0, 0;...
            0, 0, -1;...
        ];
  
    V = R*V;
  
    if isempty(handle)
      
        handle = patch('Vertices', V', 'Faces', F,...
            'FaceVertexCData',patchcolors,'FaceColor','flat');
    
    else
      
        set(handle,'Vertices',V','Faces',F);
        drawnow
    
    end
end


function pts = rotate(pts, phi, theta, psi)

    % Define rotation matrix (right handed)
  
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
      
    R = R_yaw * R_pitch * R_roll;  
  
    % note that R above either leaves the vector alone or rotates
    % a vector in a left handed rotation.  We want to rotate all
    % points in a right handed rotation, so we must transpose
  
    R = R';

    % rotate vertices
  
    pts = R*pts;
  
end


function pts = translate(pts,pn,pe,pd)

  % translate vertices by pn, pe, pd

  pts = pts + repmat([pn;pe;pd], 1, size(pts,2));
  
end


% [Define aircraft vertices and faces]

function [V,F,facecolors] = defineAircraftBody

    % Parameters for drawing aircraft
    % Scale size

    size = 2;
    fuse_l1    = 7;
    fuse_l2    = 4;
    fuse_l3    = 15;
    fuse_w     = 2;
    wing_l     = 6;
    wing_w     = 20;
    tail_h     = 3;
    tailwing_w = 10;
    tailwing_l = 3;

    % Define colors

    red = [1, 0, 0];
    green = [0, 1, 0];
    blue = [0, 0, 1];
    yellow = [1, 1, 0];
    cyan = [0, 1, 1];
    magenta = [0, 1, 1];
    mycolor = [0.7, 0.7, 0.7];
    mycolor_cyan = [0.0, 0.9, 0.9];  

    % Define the vertices (physical location of vertices)

    V = [...
            -8, -12, 0;...   % pt 1
            -10, -8, 0;...    % pt 2
            -8, -4, 0;...    % pt 3
            -12, 0, 0;...     % pt 4
            -8, 4, 0;...     % pt 5
            -10, 8, 0;...     % pt 6
            -8, 12, 0;...    % pt 7
            16, 0, 0;...    % pt 8
            12, -2, 0;...   % pt 9
            12, 2, 0;...    % pt 10
            19, 0, 0;...    % pt 11
            
            -8, 0, 0; ...    % pt 12
            4, -4, 0;...   % pt 13
            4, 4, 0;...    % pt 14
            6, 0, -4;...   % pt 15
            
            -12, -3, -5;...  % pt 16
            -14, -3, -5;...  % pt 17
            -12, 3, -5;...   % pt 18
            -14, 3, -5;...   % pt 19
        ]';

    % Rescale vertices
    
    V = size * V;   

    % Define faces as a list of vertices numbered above
    
    F = [...
            1, 2, 3;...     % tail 1
            3, 4, 5;...     % tail 2
            5, 6, 7;...     % tail 3
            1, 7, 8;...     % main frame
            9, 10, 11;...   % head
            
            13, 15, 8;...   % left body 1
            3, 13, 15;...   % left body 2
            8, 15, 14;...   % right body 1
            5, 14, 15;...   % right body 2
            3, 5, 15;...    % back body
            
            16, 12, 17;...  % left main tail 1
            17, 12, 4;...  % left main tail 2
            12, 18, 19;...  % right main tail 1
            12, 4, 19;...  % right main tail 2
        ];

    facecolors = [...
            mycolor;...         
            mycolor;...         
            mycolor;...         
            mycolor;...         
            mycolor_cyan;...         
            
            mycolor;...        
            mycolor;...         
            mycolor;...         
            mycolor;...         
            mycolor;...         
            
            mycolor;...         
            mycolor_cyan;...         
            mycolor;...       
            mycolor_cyan;...        
        ];

end
  