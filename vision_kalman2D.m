

     videoFrame = snapshot(cam);
    sceneImage = rgb2gray(videoFrame);  
    
    
[Rmin_pic, Rmax_pic] = calculate_radii_pics(-Quad.Z+Correction_axe_Z, cam_resolution_y, K_opt, real_Radii);
   
   % radiusRange = [Rmin_pic Rmax_pic];
    [centers,radii, metric] = imfindcircles(sceneImage,[Rmin_pic Rmax_pic], 'ObjectPolarity','bright');

try 
    Center = centers(1,:);
    Radii = radii(1,:);
    Metric = metric(1,:);
    detected_someth = true;
catch
    detected_obj = false;    
end


if detected_someth == true
    if Metric > 0.4
        cam_Xc = Center(1);
        cam_Yc = Center(2);
        detected_obj = true;
        Radii_c = Radii;
    else
        detected_obj = false;
    end
else
    detected_obj = false; 
end
    
if detected_obj == true
    if detected_obj_ini == true
        Q= [cam_Xc; cam_Yc; 0; 0]; %initized state--it has four components: [positionX; positionY; velocityX; velocityY] of the hexbug
        Q_estimate = Q;  %estimate of initial location estimation of where the hexbug is (what we are updating)
       
        time = 0;
        detected_obj_ini = false;
    end
    
Q_loc_meas = [cam_Xc; cam_Yc]; 

end

if detected_obj == false
            time = time + 2;
        if time >= 60
            detected_obj_ini = true;
            detected_obj = false;
        end  
end

if detected_obj_ini == false
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

    if detected_obj == true
        Q_estimate = Q_estimate + K * (Q_loc_meas - C * Q_estimate);
        time = 0;
        sceneImage = insertShape(sceneImage, 'circle', [[cam_Xc cam_Yc] Radii], 'LineWidth', 3);
    else
        cam_Xc = Q_estimate(1);
        cam_Yc = Q_estimate(2);
        sceneImage = insertShape(sceneImage, 'circle', [[cam_Xc cam_Yc] Radii], 'LineWidth', 3, 'Color', 'g');
    end   
     % update covariance estimation.
    P =  (eye(4)-K*C)*P;

else
    cam_Xc = -1;
    cam_Yc = -1;
    Radii = -1;
    sceneImage = insertShape(sceneImage, 'circle', [Center_null Radii_null], 'LineWidth', 3);
end
    
     step(videoPlayer, sceneImage);
    runLoop = isOpen(videoPlayer);
