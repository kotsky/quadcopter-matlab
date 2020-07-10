      if calcu == 100     
      Quad.X_des_GF = 10;
      cccc = 1;
      end
      
      if calcu == 400
          Quad.Z_des_GF = 29;
          cccc = 2;
      end
      if Quad.Z >= 28 && calcu > 400
          Quad.Z_des_GF = 30;
          cccc = 2;
      end
     % Quad.X_des_GF = X_cam;
     % Quad.Y_des_GF = Y_cam;
  %%   
       waypoints = waypoints_calc(25, 25, 20);

length_points = length(waypoints);
Quad.X = waypoints(1,1,:);
Quad.Y = waypoints(1,2,:);
Quad.Z = waypoints(1,3,:);

symbol_S_draw(Quad.X, -Quad.Y)

Quad.X_des_GF = waypoints(length_points,1,:);
Quad.Y_des_GF = waypoints(length_points,2,:);
Quad.Z_des_GF = waypoints(length_points,3,:);

symbol_H_draw(Quad.X_des_GF, -Quad.Y_des_GF)
 

Quad.X_des_GF = waypoints(calcu,1,:);
    Quad.Y_des_GF = waypoints(calcu,2,:);
    Quad.Z_des_GF = waypoints(calcu,3,:);

    dx = Quad.X_des_GF -  Quad.X;
    dy = Quad.Y_des_GF -  Quad.Y;
    dz = Quad.Z_des_GF -  Quad.Z;
        
    
    %%
        if Quad.init == 0
        Quad.X_des_GF = waypoints(next_point,1,:);
        Quad.Y_des_GF = waypoints(next_point,2,:);
        Quad.Z_des_GF = waypoints(next_point,3,:);
        
        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;
        
        dx = abs(Quad.X_des_GF - Quad.X_prev);
        dy = abs(Quad.Y_des_GF - Quad.Y_prev);
        dz = abs(Quad.Z_des_GF - Quad.Z_prev);
    end


dx_cont = abs(Quad.X_des_GF - Quad.X);
dy_cont = abs(Quad.Y_des_GF - Quad.Y);
dz_cont = abs(Quad.Z_des_GF - Quad.Z);



if landing == 0
    if dx_cont <= 0.5 * dx && dy_cont <= 0.5 * dy

        if next_point == length_points
            Quad.X_des_GF = waypoints(next_point,1,:);
            Quad.Y_des_GF = waypoints(next_point,2,:);
            Quad.Z_des_GF = waypoints(next_point,3,:);
            landing = -1;
        else    
        next_point = next_point + 1;

        Quad.X_des_GF = waypoints(next_point,1,:);
        Quad.Y_des_GF = waypoints(next_point,2,:);
        Quad.Z_des_GF = waypoints(next_point,3,:);

        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;

        dx = abs(Quad.X_des_GF - Quad.X_prev);
        dy = abs(Quad.Y_des_GF - Quad.Y_prev);
        dz = abs(Quad.Z_des_GF - Quad.Z_prev);
        end 
    end
else
    if dx_cont <= 0.8 * dx && dy_cont <= 0.8 * dy
        Quad.Z_des_GF = waypoints(next_point,3,:) + 1;
    end
end

%%


%% Way initialize

waypoints = waypoints_calc(15, 15, 20);

length_points = length(waypoints);
Quad.X = waypoints(1,1,:);
Quad.Y = waypoints(1,2,:);
Quad.Z = waypoints(1,3,:);

symbol_S_draw(Quad.X, Quad.Y)

Quad.X_landing = waypoints(length_points,1,:);
Quad.Y_landing = waypoints(length_points,2,:);
Quad.Z_landing = waypoints(length_points,3,:);

symbol_H_draw(Quad.X_landing, -Quad.Y_landing)

Quad.X_des_GF = Quad.X_landing;
Quad.Y_des_GF = Quad.Y_landing;

Limit_X = abs(waypoints(:,1,:));
Limit_Y = abs(waypoints(:,2,:));

Quad.X_prev = Quad.X;
Quad.Y_prev = Quad.Y;
Quad.Z_prev = Quad.Z;

%Quad.t_plot(Quad.counter-1)< max(Quad.t_plot) && 

%% Run The Simulation Loop
while  landing ~= 1 
   
    % Measure Parameters (for simulating sensor errors)
      sensor_meas;
 
    % Filter Measurements
%     Kalman_phi2;
%     Kalman_theta2;
%     Kalman_psi2;
%     Kalman_Z2;
%     Kalman_X2;
%     Kalman_Y2;
    
    if abs(Quad.X) >= Limit_X(next_point) && abs(Quad.Y) >= Limit_Y(next_point)
        next_point = next_point + 1;
    end
    
    %%
    
    if Quad.Z >= 29.8
        landing = 1;
    end
    
    %%
     if abs(Quad.X) >= Limit_X(next_point) && abs(Quad.Y) >= Limit_Y(next_point)
        next_point = next_point + 1;
        Quad.Z_des_GF = waypoints(next_point,3,:);
     end
    
     %%
    