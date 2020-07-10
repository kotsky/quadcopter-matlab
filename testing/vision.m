webcamlist
cam = webcam('Logitech')

symbolImage = imread('E:/H.jpg');
symbolImage = rgb2gray(symbolImage);
symbolPoints = detectSURFFeatures(symbolImage);
[symbolFeatures, symbolPoints] = extractFeatures(symbolImage, symbolPoints);
figure;
imshow(symbolImage);
hold on;
plot(selectStrongest(symbolPoints, 20));
title('H symbol');
figure;

c_x = 0;
c_y = 0;

for idx = 1:100
sceneImage = snapshot(cam);
sceneImage = rgb2gray(sceneImage);
scenePoints = detectSURFFeatures(sceneImage);
[sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);

symbolPairs = matchFeatures(symbolFeatures, sceneFeatures);

matchedsymbolPoints = symbolPoints(symbolPairs(:, 1), :);
matchedScenePoints = scenePoints(symbolPairs(:, 2), :);
    try
[tform, inliersymbolPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matchedsymbolPoints, matchedScenePoints, 'affine');

showMatchedFeatures(symbolImage, sceneImage, inliersymbolPoints, ...
    inlierScenePoints, 'montage');
title('Putatively Matched Points (Including Outliers)');

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

    catch
    showMatchedFeatures(symbolImage, sceneImage, matchedsymbolPoints, ...
    matchedScenePoints, 'montage');
    end

end
clear('cam');
