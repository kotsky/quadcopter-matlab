webcamlist
cam = webcam('Logitech')
cam.Resolution = '320x240';
videoFrame = snapshot(cam);
frameSize = size(videoFrame);
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
runLoop = true;
numPts = 0;
frameCount = 0;

symbolImage = imread('symbol.png');
symbolImage = rgb2gray(symbolImage);
symbolPoints = detectSURFFeatures(symbolImage);
[symbolFeatures, symbolPoints] = extractFeatures(symbolImage, symbolPoints);
figure;
imshow(symbolImage);
hold on;
plot(selectStrongest(symbolPoints, 20));
title('H symbol');
hold off;
c_x = 0;
c_y = 0;
while runLoop && frameCount < 400

    % Get the next frame.
    videoFrame = snapshot(cam);
    
    frameCount = frameCount + 1;
   
    sceneImage = rgb2gray(videoFrame);
    
    scenePoints = detectSURFFeatures(sceneImage);
[sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);

symbolPairs = matchFeatures(symbolFeatures, sceneFeatures);

matchedsymbolPoints = symbolPoints(symbolPairs(:, 1), :);
matchedScenePoints = scenePoints(symbolPairs(:, 2), :);
 try
[tform, inliersymbolPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedsymbolPoints, matchedScenePoints, 'affine');

    dots = inlierScenePoints.Location;
  
 dots_x = dots(:,1);
dots_y = dots(:,2);

n_dots = length(dots_x);


for i=1:1:n_dots
    c_x = c_x + dots_x(i);
    c_y = c_y + dots_y(i);
end
c_x = c_x/n_dots
c_y = c_y/n_dots



    bboxPolygon = [dots_x(1) dots_y(1) dots_x(2) dots_y(2) dots_x(3) dots_y(3)];
    sceneImage = insertShape(sceneImage, 'Line', bboxPolygon, 'LineWidth', 3);
    step(videoPlayer, sceneImage);
    
 catch
     bboxPolygon = [30 30 120 30 30 60 120 60];
    sceneImage = insertShape(sceneImage, 'Polygon', bboxPolygon, 'LineWidth', 3);
     step(videoPlayer, sceneImage);
  
 % Display a bounding box around the detected face.  bboxPolygon = [x1 y1 x2 y2 x3 y3 x4 y4]
          
 end
    
    runLoop = isOpen(videoPlayer);
end

clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);