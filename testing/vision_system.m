
    videoFrame = snapshot(cam);
    
    frameCount = frameCount + 1;
   
    sceneImage = rgb2gray(videoFrame);
   
    radiusRange = [10 30];
    
    [centers,radii, metric] = imfindcircles(sceneImage,radiusRange, 'ObjectPolarity','bright');
    
%viscircles(centers, radii,'Color','b');
try
    Center = centers(1,:);
    Radii = radii(1,:);
    Metric = metric(1,:);

     if Metric > 0.4
         sceneImage = insertShape(sceneImage, 'circle', [Center Radii], 'LineWidth', 3);
         cam_Xc = Center(1);
         cam_Yc = Center(2);
     else
         sceneImage = insertShape(sceneImage, 'circle', [Center_null Radii_null], 'LineWidth', 3);
         cam_Xc = -1;
         cam_Yc = -1;
     end
catch
    sceneImage = insertShape(sceneImage, 'circle', [Center_null Radii_null], 'LineWidth', 3);
    cam_Xc = -1;
    cam_Yc = -1;    
end    

    step(videoPlayer, sceneImage);
    runLoop = isOpen(videoPlayer);