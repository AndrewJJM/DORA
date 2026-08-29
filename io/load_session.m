function session = load_session(filepath)
%LOAD_SESSION  Load a previously saved DORA session from a .mat file.
%
%   session = load_session(filepath)
%   session = load_session()            % interactive: lists available sessions
%
%   If called without arguments, lists all sessions in the default data
%   directory and prompts the user to choose one.
%
%   Inputs:
%     filepath - (optional) full path to a session .mat file
%
%   Outputs:
%     session  - struct with fields: bgImg_rgb, fgImg_rgb, bgImg, fgImg,
%                cleanObsMask, inflatedObsMask, detectedObstacles,
%                startPos, goalPos, path, cfg, timestamp

    if nargin < 1 || isempty(filepath)
        % Interactive mode: list available sessions
        cfg = dora_config();
        files = dir(fullfile(cfg.dataDir, 'session_*.mat'));

        if isempty(files)
            error('No saved sessions found in: %s', cfg.dataDir);
        end

        fprintf('\n=== Available Sessions ===\n');
        for i = 1:length(files)
            fprintf('  [%d] %s\n', i, files(i).name);
        end
        fprintf('\n');

        choice = input('Select session number: ');
        if isempty(choice) || choice < 1 || choice > length(files)
            error('Invalid selection.');
        end

        filepath = fullfile(cfg.dataDir, files(choice).name);
    end

    session = load(filepath);
    fprintf('Session loaded: %s\n', filepath);

    % Display summary
    if isfield(session, 'timestamp')
        fprintf('  Captured: %s\n', char(session.timestamp));
    end
    if isfield(session, 'detectedObstacles')
        fprintf('  Obstacles: %d\n', length(session.detectedObstacles));
    end
    if isfield(session, 'path') && ~isempty(session.path)
        fprintf('  Path waypoints: %d\n', size(session.path, 1));
    else
        fprintf('  Path: (none)\n');
    end
end
