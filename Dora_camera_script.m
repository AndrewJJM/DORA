% Create a video object to access the webcam
cam = webcam('Integrated Camera');
cam.Resolution ='640x480';

% Capture a snapshot from the webcam
snapshot = snapshot(cam);

% Display the snapshot in a figure
figure;
imshow(snapshot);
title('Webcam Snapshot');

% Clear the webcam object
clear cam;