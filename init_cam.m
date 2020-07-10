webcamlist
cam = webcam('Logitech')
cam.Resolution = '320x240';
cam_resolution_x = 320;
cam_resolution_y = 240;
real_Radii = 1.888/2; %cm
K_opt = 1.5; %F/b = K

videoFrame = snapshot(cam);
frameSize = size(videoFrame);
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
runLoop = true;
numPts = 0;
time = 0;

Center_null = [1 1];
Radii_null = 1;
Metric = 1;
detected_someth = false;
detected_obj = false;
detected_obj_ini = true;

