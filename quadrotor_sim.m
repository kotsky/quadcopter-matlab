%-----------------------------------------------------------------------%
%                                                                       %
%   This script simulates quadrotor dynamics and implements a control   %                             
%   algrotihm.                                                          %
%   Developed by: Ruslan Kotvytskyi                                     %
%                                                                       %
%                                                                       %
%-----------------------------------------------------------------------%

% Add Paths
addpath utilities

%% Initialize Workspace
clear all;
close all;
clc;

FLY_MODE = 3; %1 = FOLLOW, 2 = LANDING, 3 = TESTING

global Quad;

calcu = 0;
Quad.Correction_axe_Z = 50;
Correction_axe_Z = Quad.Correction_axe_Z;

%% Initialize the plot
init_plot(FLY_MODE);
plot_quad_model;

%% Initialize Variables
quad_variables;
quad_dynamics_nonlinear; 

%%
%symbol_S_draw(Quad.X, -Quad.Y)
%Quad.t_plot(Quad.counter-1)< max(Quad.t_plot) && landing ~= 1


switch FLY_MODE
    case 1
      
        %% Initialize Variables of vision system
        init_cam;
        kalman2D_ini;
        init = 0;
        %% Init drawing part

        %Target part
        R = 1;
        Width_target = 2;

        circle_H = rectangle('Position',[-R -R 2*R 2*R],'Curvature',[1 1], 'LineWidth', Width_target, 'EdgeColor', 'b')

        H1 = line([0 0],[-0.8*R 0.8*R],'LineWidth', Width_target, 'Color', 'b')
        H2 = line([-0.8*R 0.8*R],[0 0],'LineWidth',Width_target, 'Color', 'b')

        %Square cam part
        Width_square = 1;
        Radii = 15;
        GSD = real_Radii / Radii;

        X_square = K_opt * cam_resolution_x * GSD;
        Y_square = K_opt * cam_resolution_y * GSD;

        Xc_tr = X_square/2;
        Yc_tr = Y_square/2;

        L1 = line([-X_square/2 0],[-Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L2 = line([X_square/2 0],[-Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L3 = line([-X_square/2 0],[Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L4 = line([X_square/2 0],[Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')   
        square_cam_real = rectangle('Position',[-X_square/2 -Y_square/2 X_square Y_square], 'EdgeColor', 'black', 'LineWidth', Width_square)
        
        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;
        
        %Track part

        Hor = line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[0 0],'color','b')
        All_track = line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[-Quad.Z+Quad.Correction_axe_Z -Quad.Z_prev+Quad.Correction_axe_Z],'color','r')
        
    case 2
        
                %% Initialize Variables of vision system
        init_cam;
        kalman2D_ini;
        init = 0;
        
        %%Additional values
        %landing = 0;  %landing part
        next_point = 2;
        true_obj = 0;
        detect_lines = false;
        LANDING_MODE = 1;
        
         %Square cam part
        Width_square = 1;

        X_square = (-Quad.Z + Correction_axe_Z) * 1.25 / K_opt;
        Y_square = (-Quad.Z + Correction_axe_Z) / K_opt;

        Xc_tr = X_square/2;
        Yc_tr = Y_square/2;

        L1 = line([-X_square/2 0],[-Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L2 = line([X_square/2 0],[-Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L3 = line([-X_square/2 0],[Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')
        L4 = line([X_square/2 0],[Y_square/2 0],[0 20],'LineWidth', Width_square, 'Color', 'black')   
        square_cam_real = rectangle('Position',[-X_square/2 -Y_square/2 X_square Y_square], 'EdgeColor', 'black', 'LineWidth', Width_square)
        
        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;
        
    case 3
        
        Quad.X_des_GF = 10; 
        Quad.Y_des_GF = -10;
        Quad.Z_des_GF = Quad.Z_init - 10;
        
end
        runLoop = true;
        
        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;
        

%% Run The Simulation Loop
while  runLoop
   
    % Measure Parameters (for simulating sensor errors)
      sensor_meas;
 
    % Filter Measurements
%     Kalman_phi2;
%     Kalman_theta2;
%     Kalman_psi2;
%     Kalman_Z2;
%     Kalman_X2;
%     Kalman_Y2;

    %%
    % Implement Controller // control system part
    position_PID; 
    attitude_PID;
    rate_PID;

    % Calculate Desired Motor Speeds
    quad_motor_speed;
    
    % Update Position With The Equations of Motion
    quad_dynamics_nonlinear;    
    
    %% 
    % Plot the Quadrotor's Position
    if(mod(Quad.counter,3)==0)
        plot_quad

        % Plot the Quadrotor's track
        
        switch FLY_MODE
            case 1
                 set(Hor, 'XData', [0 Quad.X], 'YData', [0 -Quad.Y] )
                 set(All_track, 'XData', [0 Quad.X], 'YData', [0 -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
            case 2
         % Plot the Quadrotor's track
                line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[-Quad.Z+Quad.Correction_axe_Z -Quad.Z_prev+Quad.Correction_axe_Z],'color','r')
                line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[0 0],'color','b')
            case 3
                line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[-Quad.Z+Quad.Correction_axe_Z -Quad.Z_prev+Quad.Correction_axe_Z],'color','r')
                line([Quad.X Quad.X_prev],[-Quad.Y -Quad.Y_prev],[0 0],'color','b')
        end
         
   

        Quad.X_prev = Quad.X;
        Quad.Y_prev = Quad.Y;
        Quad.Z_prev = Quad.Z;

        Quad.counter;      
        drawnow 
    end

    %%
    %vision system part
switch FLY_MODE     
    
    case 1  %following mode
        if mod(calcu,12) == 0  %fps divide
          vision_kalman2D; 

           if cam_Xc == -1
               Quad.X_des_GF = Quad.X_prev; 
               Quad.Y_des_GF = Quad.Y_prev;
               Quad.Z_des_GF = Quad.Z_prev;
           else

               GSD = real_Radii / Radii;

               X_square = cam_resolution_x * GSD;
               Y_square = cam_resolution_y * GSD;

           Xc = cam_Xc * GSD;
           Yc = cam_Yc * GSD;

           Xc_tr = Xc - X_square/2;
           Yc_tr = Yc - Y_square/2;

           Zc_tr = K_opt * cam_resolution_y * GSD;

           Quad.X_des_GF = -Xc_tr; 
           Quad.Y_des_GF = -Yc_tr; 
           Quad.Z_des_GF = Quad.Correction_axe_Z - Zc_tr;

               if mod(calcu,24) == 0
                   %draw part
                        %draw cam square    
                        set(L1, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L2, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L3, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L4, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(square_cam_real,'Position',[-X_square/2+Quad.X -Y_square/2-Quad.Y X_square Y_square])
               end
           end   
        end
        
    case 2 %landing mode
        
        switch LANDING_MODE % 1 = LANDING_MODE_DETECT; 2 = LANDING_MODE_HIGH_HEIGHT; 3 = LANDING_MODE_LOW_HEIGHT
            
            case 1
                  if mod(calcu,12) == 0  %fps divide
                      vision_kalman2D; 

                       if cam_Xc == -1
                           %Quad.X_des_GF = Quad.X_prev; 
                           %Quad.Y_des_GF = Quad.Y_prev;
                           %Quad.Z_des_GF = Quad.Z_prev;
                           true_obj = 0;
                       else

                           GSD = real_Radii / Radii;

                           X_square = cam_resolution_x * GSD;
                           Y_square = cam_resolution_y * GSD;

                       Xc = cam_Xc * GSD;
                       Yc = cam_Yc * GSD;

                       Xc_tr = Xc - X_square/2;
                       Yc_tr = Yc - Y_square/2;

                     

                       true_obj = true_obj + 1;
                       end

           Zc_tr = K_opt * cam_resolution_y * GSD;

           %Quad.X_des_GF = -Xc_tr; 
           %Quad.Y_des_GF = -Yc_tr; 
           Quad.Z_des_GF = Quad.Correction_axe_Z - Zc_tr;

               if mod(calcu,24) == 0
                   %draw part
                        %draw cam square    
                        set(L1, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L2, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L3, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L4, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(square_cam_real,'Position',[-X_square/2+Quad.X -Y_square/2-Quad.Y X_square Y_square])
               end

                       if true_obj == 40
                           LANDING_MODE = 2;
                           symbol_H_draw(Xc_tr, -Yc_tr, 'b')
                       end

                  end
                  
            case 2
                %vision_kalman2D;
                
                    X_square = (-Quad.Z + Correction_axe_Z) * 1.25 / K_opt;
                    Y_square = (-Quad.Z + Correction_axe_Z) / K_opt;

                       

                       %Quad.X_des_GF = -Xc_tr; 
                       %Quad.Y_des_GF = -Yc_tr; 

                      

                        Zc_tr = (-Quad.Z + Correction_axe_Z);

               if mod(calcu,24) == 0
                   %draw part
                        %draw cam square    
                        set(L1, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L2, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L3, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L4, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(square_cam_real,'Position',[-X_square/2+Quad.X -Y_square/2-Quad.Y X_square Y_square])
               end
                
                
                    % Track calculation
                    [waypoints, Quad.psi_des] = waypoints_calc(Xc_tr, Yc_tr, 20, Correction_axe_Z);

                    length_points = length(waypoints);


                    if next_point <= length_points

                        Quad.X_des_GF = waypoints(next_point,1,:);         % desired value of X in Global frame
                        Quad.Y_des_GF = waypoints(next_point,2,:);         % desired value of Y in Global frame
                        Quad.Z_des_GF = waypoints(next_point,3,:);
                        dx = abs(Quad.X_des_GF - Quad.X_prev);
                        dy = abs(Quad.Y_des_GF - Quad.Y_prev);

                        df = sqrt( dx^2 + dy^2 );
                        

                        Limit = waypoints(next_point,4,:);

                        if df <= 0.8*Limit
                            next_point = next_point + 1;
                        end
                    else
                        dz = abs(Correction_axe_Z - Quad.Z);
                        if dz <= 2.5
                            LANDING_MODE = 3;
                            
                            set(L1, 'Color', 'red')
                            set(L2, 'Color', 'red')
                            set(L3, 'Color', 'red')
                            set(L4, 'Color', 'red')
                            set(square_cam_real,'Color', 'red')
                            
                            
                            %init for nest stage (lines vision system)
                            detected_obj = false;
                            detected_obj_ini = true;
                            
                            u = 0.1; % define acceleration magnitude
                            HexAccel_noise_mag = 0.06; %process noise: the variability in how fast the Hexbug is speeding up (stdv of acceleration: meters/sec^2)
                            
                            Ez = [tkn_x 0; 0 tkn_y];
                            Ex = [dt^4/4 0 dt^3/2 0; ...
                                0 dt^4/4 0 dt^3/2; ...
                                dt^3/2 0 dt^2 0; ...
                                0 dt^3/2 0 dt^2].*HexAccel_noise_mag^2; % Ex convert the process noise (stdv) into covariance matrix
                            P = Ex; % estimate of initial Hexbug pos 
                            
                        end
                    end

            case 3
                if mod(calcu,12) == 0  %fps divide
                    vision_lines;
                end
                
                if mod(calcu,24) == 0
                   %draw part
                        %draw cam square    
                        set(L1, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L2, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [-Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L3, 'XData', [-X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(L4, 'XData', [X_square/2+Quad.X Quad.X], 'YData', [Y_square/2-Quad.Y -Quad.Y], 'ZData', [0 -Quad.Z+Quad.Correction_axe_Z] )
                        set(square_cam_real,'Position',[-X_square/2+Quad.X -Y_square/2-Quad.Y X_square Y_square])
               end

                    if detected_obj_ini == false
                        Quad.Z_des_GF = Correction_axe_Z;
                    end

        end
        
        
        
        
end
    init = 1;
    calcu = calcu + 1;
    Quad.init = 1;  %Ends initialization after first simulation iteration
    
    
    if FLY_MODE == 3
        if calcu == 1000    %test finish
           runLoop = false; 
           plot_data
        end
    else
        if Quad.Z >= Correction_axe_Z - 0.2   %landing on 20cm on the ground
            runLoop = false;
            plot_data
        end
    end
    
    
    
end

%% Plot Data
%plot_data


