function filepath = save_session(cfg, bgImg_rgb, fgImg_rgb, ...
                                 cleanObsMask, inflatedObsMask, ...
                                 detectedObstacles, startPos, goalPos, path)
%SAVE_SESSION  Persist the full pipeline state to a timestamped .mat file.
%
%   filepath = save_session(cfg, bgImg_rgb, fgImg_rgb, cleanObsMask, ...
%                           inflatedObsMask, detectedObstacles, ...
%                           startPos, goalPos, path)
%
%   Saves all intermediate data into:
%     <cfg.dataDir>/session_YYYYMMDD_HHMMSS.mat
%
%   The saved struct contains:
%     bgImg_rgb, fgImg_rgb    — original RGB images
%     bgImg, fgImg            — grayscale versions
%     cleanObsMask            — exact obstacle mask
%     inflatedObsMask         — inflated obstacle mask
%     detectedObstacles       — regionprops struct array
%     startPos, goalPos       — [x,y] positions
%     path                    — Nx2 waypoints (may be empty)
%     cfg                     — config used for this session
%     timestamp               — datetime of save
%
%   Outputs:
%     filepath - full path to the saved .mat file

    % Ensure data directory exists
    if ~exist(cfg.dataDir, 'dir')
        mkdir(cfg.dataDir);
    end

    % Build timestamped filename
    ts = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<TNOW1,DATST>
    filename = sprintf('session_%s.mat', ts);
    filepath = fullfile(cfg.dataDir, filename);

    % Derive grayscale versions for convenience
    bgImg = rgb2gray(bgImg_rgb);
    fgImg = rgb2gray(fgImg_rgb);

    % Build session struct
    session.bgImg_rgb         = bgImg_rgb;
    session.fgImg_rgb         = fgImg_rgb;
    session.bgImg             = bgImg;
    session.fgImg             = fgImg;
    session.cleanObsMask      = cleanObsMask;
    session.inflatedObsMask   = inflatedObsMask;
    session.detectedObstacles = detectedObstacles;
    session.startPos          = startPos;
    session.goalPos           = goalPos;
    session.path              = path;
    session.cfg               = cfg;
    session.timestamp         = datetime('now');

    save(filepath, '-struct', 'session');
    fprintf('Session saved: %s\n', filepath);
end
