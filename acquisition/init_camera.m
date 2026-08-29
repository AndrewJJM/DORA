function cam = init_camera(cfg)
%INIT_CAMERA  Initialize the camera device based on config.
%
%   cam = init_camera(cfg) sets up either MATLAB Mobile (back camera)
%   or a local webcam, depending on cfg.useMobileCamera.
%
%   Inputs:
%     cfg  - config struct from dora_config()
%
%   Outputs:
%     cam  - camera object ready for snapshot()

    if cfg.useMobileCamera
        disp('Connecting to MATLAB Mobile camera...');
        m = mobiledev;
        % TODO: Add exception handling for missing device
        cam = camera(m, 'back');
        cam.Resolution = cfg.camResolution;
    else
        disp('Connecting to local webcam...');
        cam = webcam();
    end

    disp('Camera initialized.');
end
