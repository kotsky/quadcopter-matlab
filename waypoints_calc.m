function [ waypoints , psi_dir_end ] = waypoints_calc( x_coord, y_coord, altitude, Correction_axe_Z )

if x_coord < 0
    pos_x = -1;
    x_coord = abs(x_coord);
else
    pos_x = 1;
end

if y_coord < 0
    pos_y = -1;
    y_coord = abs(y_coord);
else
    pos_y = 1;
end

altitude = altitude - 2; %????? ? 2 ????? ??? ??????????

psi_dir = atan(y_coord/x_coord)*(-pos_y);

if pos_x == -1
   psi_dir =  pi - psi_dir;
end

%psi_dir_end = psi_dir * (180/pi);
psi_dir_end = psi_dir;
xy_coord = sqrt(x_coord^2 + y_coord^2);

%Est = 4;
%E = ( pi * xy_coord * altitude + ( xy_coord - altitude )^2 ) / ( xy_coord + altitude);
%St = E / Est;
%phi_counter = pi/(2*St);
phi_counter = pi/9;
    phi_polar_VS = 0:phi_counter:pi/2;
    p_polar_VS = xy_coord*altitude./sqrt((xy_coord^2) * (sin(phi_polar_VS)).^2 + (altitude^2) * (cos(phi_polar_VS)).^2 );
    z_waypoints_VS = p_polar_VS .* sin(phi_polar_VS);
    xy_waypoints_VS = p_polar_VS .* cos(phi_polar_VS);

    y_waypoints_VS = xy_waypoints_VS.* ( y_coord / xy_coord);
    x_waypoints_VS = xy_waypoints_VS.* ( x_coord / xy_coord);

    n = length(xy_waypoints_VS);

    waypoints_T = zeros(n,4);

    waypoints_T(1:n,1,:) = x_waypoints_VS;
    waypoints_T(1:n,2,:) = y_waypoints_VS;
    waypoints_T(1:n,3,:) = z_waypoints_VS;
    waypoints_T(1,:,:) = [ x_coord y_coord 0 0 ];
    waypoints_T(n,:,:) = [ 0 0 altitude 0 ];

    waypoints_T(1:n,1,:) = waypoints_T(1:n,1,:) .* pos_x;
    waypoints_T(1:n,2,:) = waypoints_T(1:n,2,:) .* pos_y;
    waypoints_T(1:n,2,:) = waypoints_T(1:n,2,:) .* (1);
    waypoints_T(1:n,3,:) = waypoints_T(1:n,3,:) - Correction_axe_Z;
    waypoints_T(1:n,3,:) = waypoints_T(1:n,3,:).* (-1);
    
    waypoints = waypoints_T;

    for i = 1:n
        waypoints(i,:,:) = waypoints_T(n+1-i, :, :);   
        waypoints(i,3,:) = waypoints(i,3,:) - 2;
        
    end
    sum = 0;
    for i = 2:n
       sum = sum + waypoints(i-1,4,:);
       waypoints(i,4,:) = sqrt(waypoints(i,1,:)^2 + waypoints(i,2,:)^2) - sum;
    end
    
    %waypoints(1:n,3,:) = waypoints(1:n,3,:).* (-1);
    
end

