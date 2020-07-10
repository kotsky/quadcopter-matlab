%webcamlist
%cam = webcam('Logitech')

videoFrame = snapshot(cam);
frameSize = size(videoFrame);
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
runLoop = true;
numPts = 0;
frameCount = 0;


while runLoop && frameCount < 300
    frameCount = frameCount + 1;
     videoFrame = snapshot(cam);
    
    frameCount = frameCount + 1;
   
    sceneImage = rgb2gray(videoFrame);
   
    sceneImage = imbinarize(sceneImage);
    sceneImage = bwareaopen(sceneImage,30);
    
    [H,theta,rho] = hough(sceneImage);
    
    P = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
    x = theta(P(:,2)); 
    y = rho(P(:,1));
    
    lines = houghlines(sceneImage,theta,rho,P,'FillGap',5,'MinLength',7);
    
    step(videoPlayer, sceneImage);
    
    
    
end