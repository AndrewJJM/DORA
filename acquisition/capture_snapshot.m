function img = capture_snapshot(cam)
%CAPTURE_SNAPSHOT  Capture a single frame from the camera.
%
%   img = capture_snapshot(cam) returns an RGB image from the given
%   camera object.  This thin wrapper exists so snapshot logic (retry,
%   logging, pre-processing) can be changed in one place.
%
%   Inputs:
%     cam  - camera object from init_camera()
%
%   Outputs:
%     img  - RGB uint8 image

    img = snapshot(cam, 'immediate');
end
