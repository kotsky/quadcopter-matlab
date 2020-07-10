
videoFrame = snapshot(cam);
   
scene_img_gray = rgb2gray(videoFrame);

BW = edge(scene_img_gray,'sobel');

[H,theta,rho] = hough(BW);

L_p = houghpeaks(H,5,'threshold',ceil(0.6*max(H(:))));

lines = houghlines(BW,theta,rho,L_p,'FillGap',10,'MinLength',15);
q_lines = 0;
data_lines = [];
last_direction = -1000;
angle = 0;
min_x = -1;

try

        for k = 1:length(lines)
           xy = [lines(k).point1; lines(k).point2];

           %len = norm(lines(k).point1 - lines(k).point2);
         q_lines = q_lines + 1;
          scene_img_gray = insertShape(scene_img_gray, 'Line', [xy(1,1) xy(1,2) xy(2,1) xy(2,2)],'Color', 'g', 'LineWidth', 3);

          data_lines(q_lines).coord  = [xy(1,1) xy(1,2) xy(2,1) xy(2,2)];

           direction = lines(k).rho;

           if direction ~= last_direction
               last_direction = direction;   
               perpend = lines(k).theta; 
           end
           data_lines(q_lines).angle_th = perpend;   

           if k == length(lines)


                    sum_x = 0;
                    sum_y = 0;
                    x_corr = 0;
                    y_corr = 0;
                    perp_quat = 0;
                    parallel_quat = 0;

                   for i = 1:length(lines)-1

                       diff_angle = abs(data_lines(i).angle_th - data_lines(i+1).angle_th);

                       y1 = data_lines(i).coord(2);
                       y2 = data_lines(i).coord(4);
                       x1 = data_lines(i).coord(1);
                       x2 = data_lines(i).coord(3);

                       y3 = data_lines(i+1).coord(2);
                       y4 = data_lines(i+1).coord(4);
                       x3 = data_lines(i+1).coord(1);
                       x4 = data_lines(i+1).coord(3);

                           bf = (y1 * x2 - x1 * y2)/(x2 - x1);
                           bg = (y3 * x4 - x3 * y4)/(x4 - x3);
                           af = (y2 - y1)/(x2 - x1);
                           ag = (y4 - y3)/(x4 - x3);

                       if diff_angle >= 0 && diff_angle <= 3
                           %parallel
                           parallel_quat = parallel_quat + 1;

                           if abs(data_lines(i).angle_th) >= 85 && abs(data_lines(i).angle_th) <= 95
                               x_corr = x_corr + 0;
                               y_corr = y_corr + (y1 + y2)/2;
                           elseif  abs(data_lines(i).angle_th) >= 0 && abs(data_lines(i).angle_th) <= 5
                               x_corr = x_corr + (x1 + x2)/2;
                               y_corr = y_corr + 0;
                           else
                               distance_perp = abs(bf - bg) * cos(data_lines(i).angle_th * pi/180);

                               x_corr = x_corr + abs(distance_perp) * sin(data_lines(i).angle_th * pi/180);
                               y_corr = y_corr + abs(distance_perp) * cos(data_lines(i).angle_th * pi/180);
                           end


                       elseif diff_angle >= 87 && diff_angle <= 93
                           %perpend
                           perp_quat = perp_quat + 1;

                           if abs(data_lines(i).angle_th) >= 85 && abs(data_lines(i).angle_th) <= 95
                               x_con = (x3 + x4)/2;
                               y_con = (y1 + y2)/2;
                           elseif  abs(data_lines(i).angle_th) >= 0 && abs(data_lines(i).angle_th) <= 5
                               x_con = (x1 + x2)/2;
                               y_con = (y3 + y4)/2;
                           else
                               x_con = (bg - bf)/(af - ag);
                               y_con = x_con*af + bf;
                           end

                                    sum_x = sum_x + x_con;
                                    sum_y = sum_y + y_con;

                       end


                   end

                   if perp_quat == 0
                       average_x = x_corr;
                       average_y = y_corr;
                       detected_obj = false;


                   else
                       %average_x = sum_x/(perp_quat) + 0.3 * x_corr/(parallel_quat+1);
                       %average_y = sum_y/(perp_quat) + 0.3 * y_corr/(parallel_quat+1);
                       average_x = sum_x/(perp_quat);
                       average_y = sum_y/(perp_quat);
                   end

                   average_x = round(average_x);
                   average_y = round(average_y);
                   
                   if average_x >= 300 || average_y >= 220
                      average_x = 320/2;
                      average_y = 240/2;                   
                   end


                   if isnan(average_x) && average_x == inf
                        average_x = 320/2;
                        average_y = 240/2;
                   end


                   scene_img_gray = insertShape(scene_img_gray, 'circle', [[average_x average_y] 5], 'LineWidth', 3, 'Color', 'blue');

                   if detected_obj_ini == true
                   Q= [average_x; average_y; 0; 0]; %initized state--it has four components: [positionX; positionY; velocityX; velocityY] of the hexbug
                   Q_estimate = Q;  %estimate of initial location estimation of where the hexbug is (what we are updating)     
                    detected_obj_ini = false;
                   end

                   Q_loc_meas = [average_x; average_y]; 


                        % Predict next state of the Hexbug with the last state and predicted motion.
                        Q_estimate = A * Q_estimate + B * u;
                        %predic_state = [predic_state; Q_estimate(1)] ;
                        %predict next covariance
                        P = A * P * A' + Ex;
                        %predic_var = [predic_var; P] ;
                        % predicted Ninja measurement covariance
                        % Kalman Gain
                        K = P*C'*inv(C*P*C'+Ez);
                        % Update the state estimate.


                Q_estimate = Q_estimate + K * (Q_loc_meas - C * Q_estimate);
                time = 0;

                average_x = Q_estimate(1);
                average_y = Q_estimate(2);
                scene_img_gray = insertShape(scene_img_gray, 'circle', [[average_x average_y] 5], 'LineWidth', 3, 'Color', 'red');

             % update covariance estimation.
            P =  (eye(4)-K*C)*P;




            end


        end
catch
    if detected_obj_ini == false
        Q_loc_meas = [320/2; 240/2]; 


                        % Predict next state of the Hexbug with the last state and predicted motion.
                        Q_estimate = A * Q_estimate + B * u;
                        %predic_state = [predic_state; Q_estimate(1)] ;
                        %predict next covariance
                        P = A * P * A' + Ex;
                        %predic_var = [predic_var; P] ;
                        % predicted Ninja measurement covariance
                        % Kalman Gain
                        K = P*C'*inv(C*P*C'+Ez);
                        % Update the state estimate.


                Q_estimate = Q_estimate + K * (Q_loc_meas - C * Q_estimate);
                time = 0;

                average_x = Q_estimate(1);
                average_y = Q_estimate(2);
                scene_img_gray = insertShape(scene_img_gray, 'circle', [[average_x average_y] 5], 'LineWidth', 3, 'Color', 'blue');

             % update covariance estimation.
            P =  (eye(4)-K*C)*P;

    end
end
        
        
%scene_img_gray = insertShape(scene_img_gray, 'Line', [xy_long(1,1) xy_long(1,2) xy_long(2,1) xy_long(2,2)], 'LineWidth', 3);
step(videoPlayer, scene_img_gray);
    runLoop = isOpen(videoPlayer);
    max_len = 0;
    direction = 0;

