%DORA_SETUP  Add all DORA subdirectories to the MATLAB path.
%   Run this script once per MATLAB session (or add it to your startup.m)
%   to make all DORA modules available on the path.

rootDir = fileparts(mfilename('fullpath'));

addpath(rootDir);
addpath(fullfile(rootDir, 'acquisition'));
addpath(fullfile(rootDir, 'detection'));
addpath(fullfile(rootDir, 'pathfinding'));
addpath(fullfile(rootDir, 'visualization'));
addpath(fullfile(rootDir, 'io'));
addpath(fullfile(rootDir, 'legacy'));

% Create data directory if it doesn't exist
dataDir = fullfile(rootDir, 'data');
if ~exist(dataDir, 'dir')
    mkdir(dataDir);
end

fprintf('DORA paths configured. Data directory: %s\n', dataDir);

clear rootDir dataDir;
