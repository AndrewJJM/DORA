% =========================================================================
% DORA: Bird's Eye View Pathfinding & ESP32 Control
% =========================================================================
% This script connects to a smartphone camera, extracts a map, identifies
% the robot, goal, and obstacles via background subtraction, computes a path,
% and sends differential drive commands to an ESP32 via UDP.
% 1. Image Acquisition (MATLAB Mobile or Webcam)
% 2. Homomorphic Filtering (Illumination Correction)
% 3. Adaptive Thresholding & Morphology
% 4. ArUco Marker Anchor Detection
% 5. Rectangular Obstacle Detection
% 6. Bird's-Eye-View (Inverse Perspective Mapping)
% 7. Binary Occupancy Grid Mapping
% =========================================================================

clear; clc; close all;

%% Configuration Setup
USE_MOBILE_CAMERA = true; % Utilizzo MATLAB Mobile

% IDs dei marker come mostrato nel tuo workspace
START_MARKER_ID = 0; 
GOAL_MARKER_ID  = 1; 
ROBOT_MARKER_ID = 0;

ROBOT_RADIUS_PX = 10; % Raggio di gonfiaggio attorno all'ostacolo

imgWidth = 480;
imgHeight = 640;

if USE_MOBILE_CAMERA
    disp('Connessione alla fotocamera di MATLAB Mobile...');
    m = mobiledev;
    %TODO Add exception in case of no device        
    cam = camera(m, 'back');
    cam.Resolution = '640x480';
else
    disp('Connessione alla webcam locale...');
    cam = webcam();
end

hFig = figure('Name', 'BEV A* Pathfinding', 'Position', [100, 100, 1000, 600]);
disp('Inizio elaborazione. Chiudi la finestra per fermare il programma.');

%% Obstacle detection

disp('STEP 1: BACKGROUND CAPTURE');
disp('Please CLEAR THE FLOOR (No robot, no markers, no obstacles).');
disp('Capturing background in 3 seconds...');
pause(3);
bgImg_rgb = snapshot(cam, 'immediate');
bgImg = rgb2gray(bgImg_rgb);
bgImg = imflatfield(medfilt2(bgImg, [5 5]), 30); 
disp('Background captured.');

% STREAMING_CHUNK:Capturing and subtracting obstacles...
disp('---------------------------------------------------------');
disp('STEP 2: OBSTACLE CAPTURE');
disp('Please PLACE OBSTACLES ONLY (Do not place the robot or markers yet).');
disp('press any key to take the picture...');
pause;
fgImg_rgb = snapshot(cam, 'immediate');
cleanImg = rgb2gray(fgImg_rgb);
cleanImg = imflatfield(medfilt2(cleanImg, [5 5]), 30);

% --- OLD Image Cleaning (consider removing) ---
cleanImg = medfilt2(cleanImg, [5 5]);
cleanImg = imflatfield(cleanImg, 30);

% Perform Background Subtraction
diffImg = imabsdiff(cleanImg, bgImg);

% Thresholding: A slightly higher threshold cuts out soft shadow gradients
diffThreshold = 180; % Tweak this number (0-255) to ignore shadows
bwObs = diffImg > diffThreshold;

% Shadow Reduction & Cleanup
% 1. imopen (erosion then dilation) removes weak, attached shadow boundaries 
%    without permanently shrinking the main object.
se_clean = strel('disk', 2);
bwObs = imopen(bwObs, se_clean); 

% 2. Remove disconnected tiny specks (noise)
bwObs = bwareaopen(bwObs, 200); 

% 3. Solidify the interior of the shapes (in case the tops of obstacles 
%    looked identical to the floor)
finalObsMask = imfill(bwObs, 'holes');

% Generate the static occupancy map
% Inflate the obstacles by the robot's radius
se_inflate = strel('disk', ROBOT_RADIUS_PX);
inflatedObsMask = imdilate(finalObsMask, se_inflate);

% Create Occupancy Map (Direct 1:1 Pixel Mapping) once!
map = binaryOccupancyMap(inflatedObsMask);

% Initialize A* Planner once!
planner = plannerAStarGrid(map);

disp('---------------------------------------------------------');
disp('STATIC MAP GENERATED!');
disp('Please place the ROBOT and GOAL MARKERS.');
disp('press any key to take the picture...');
pause;

disp('---------------------------------------------------------');
disp('STEP 3: PATHFINDING');
disp('Starting pathfinding');

pathImg = snapshot(cam, 'immediate');

try
    [markerIDs, markerLocs] = readArucoMarker(pathImg, 'DICT_4X4_50');
catch
    error('Could not detect ArUco markers. Please restart and ensure markers are visible.');
end

startPos = [];
goalPos = [];

for i = 1:length(markerIDs)
    corners = markerLocs(:,:,i);
    centerX = mean(corners(:,1));
    centerY = mean(corners(:,2));
    
    if markerIDs(i) == ROBOT_MARKER_ID
        startPos = [centerX, centerY];
    elseif markerIDs(i) == GOAL_MARKER_ID
        goalPos = [centerX, centerY];
    end
end

if isempty(startPos) || isempty(goalPos)
    error('Failed to find Robot (0) and Goal (1) markers. Restart the script.');
end

% Compute the A* Path exactly once
startPt = round(startPos);
goalPt = round(goalPos);
path = [];
try
    path = plan(planner, startPt, goalPt);
    disp('A* Path calculated successfully!');
catch
    disp('WARNING: No valid path found (Goal or Start might be inside an obstacle).');
end

disp('---------------------------------------------------------');
disp('SETUP COMPLETE. Starting live tracking loop...');
disp('Close the figure window to stop the program.');
disp('---------------------------------------------------------');

%% Main Processing Loop
while ishandle(hFig)
    
    % --- Acquisizione Immagine ---
    img = snapshot(cam, 'immediate');

    % --- B. Live Robot Tracking ---
    % Only look for the robot marker now to update its live position
    try
        [liveIDs, liveLocs] = readArucoMarker(img, 'DICT_4X4_50');
    catch
        liveIDs = []; liveLocs = [];
    end
    
    currentRobotPos = [];
    if ~isempty(liveLocs)
        for i = 1:length(liveIDs)
            if liveIDs(i) == ROBOT_MARKER_ID
                corners = liveLocs(:,:,i);
                currentRobotPos = [mean(corners(:,1)), mean(corners(:,2))];
                break; % Found the robot, no need to check other markers
            end
        end
    end
    
    % 1. Draw Exact Obstacle Boundaries (Static)
    obsBoundaries = bwboundaries(finalObsMask);
    for k = 1:length(obsBoundaries)
        b = obsBoundaries{k};
        plot(b(:,2), b(:,1), 'r-', 'LineWidth', 2);
    end
    
    % 2. Draw Pre-computed Inflated Safety Boundaries (Static)
    boundaries = bwboundaries(inflatedObsMask);
    for k = 1:length(boundaries)
        boundary = boundaries{k};
        plot(boundary(:,2), boundary(:,1), 'y--', 'LineWidth', 1);
    end
    
    % 3. Draw Initial Start and Static Goal Points
    plot(startPos(1), startPos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    text(startPos(1)-20, startPos(2)-20, 'START', 'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');
    
    plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 3);
    text(goalPos(1)-20, goalPos(2)-20, 'GOAL', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
        
    % 4. Draw the Static A* Path
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'c-', 'LineWidth', 4);
    end
    
    % 5. Draw the LIVE Robot Position
    if ~isempty(currentRobotPos)
        plot(currentRobotPos(1), currentRobotPos(2), 'mo', 'MarkerSize', 12, 'LineWidth', 4);
        text(currentRobotPos(1)+15, currentRobotPos(2), 'ROBOT', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    title('Real-Time Tracking along Static A* Path');
    hold off;
    
    drawnow; % Force UI update
    
    
    % --- E. Mappatura & Pathfinding A* CORRETTO ---
    se = strel('disk', ROBOT_RADIUS_PX);
    inflatedObsMask = imdilate(finalObsMask, se);
    
    % TRASPOSIZIONE ( ' ): Allinea il sistema [Righe, Colonne] a [X, Y] per A*
    map = binaryOccupancyMap(inflatedObsMask');
    
    path = [];
    if ~isempty(startPos) && ~isempty(goalPos)
        planner = plannerAStarGrid(map);
        
        startPt = round(startPos);
        goalPt = round(goalPos);
        
        % Assicura che i punti non superino mai i limiti dello schermo
        startPt(1) = max(1, min(startPt(1), imgWidth));
        startPt(2) = max(1, min(startPt(2), imgHeight));
        goalPt(1)  = max(1, min(goalPt(1), imgWidth));
        goalPt(2)  = max(1, min(goalPt(2), imgHeight));
        
        try
            path = plan(planner, startPt, goalPt);
        catch
            % Nessun percorso valido se il goal è intrappolato dentro un ostacolo
        end
    end
    
    % --- Visualizzazione ---
    clf(hFig);
    imshow(img);
    hold on;
    
    % 1. Disegna Bounding Box Ostacoli (Rettangoli Rossi)
    % for k = 1:size(rectObstacles, 1)
    %     rectangle('Position', rectObstacles(k,:), 'EdgeColor', 'r', 'LineWidth', 2);
    % end
    
    % 2. Disegna i bordi di sicurezza espansi per il raggio del robot (Tratteggio Giallo)
    boundaries = bwboundaries(inflatedObsMask);
    for k = 1:length(boundaries)
        boundary = boundaries{k};
        plot(boundary(:,2), boundary(:,1), 'y--', 'LineWidth', 1);
    end
    
    % 3. Disegna i punti di START e GOAL
    if ~isempty(startPos)
        plot(startPos(1), startPos(2), 'go', 'MarkerSize', 10, 'LineWidth', 3);
        text(startPos(1)-20, startPos(2)-20, 'START', 'Color', 'g', 'FontSize', 12, 'FontWeight', 'bold');
    end
    if ~isempty(goalPos)
        plot(goalPos(1), goalPos(2), 'ro', 'MarkerSize', 10, 'LineWidth', 3);
        text(goalPos(1)-20, goalPos(2)-20, 'GOAL', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    % 4. Disegna il percorso dell'A* (Linea Ciano)
    if ~isempty(path)
        plot(path(:,1), path(:,2), 'c-', 'LineWidth', 4);
    end

    % 5. Draw the LIVE Robot Position
    if ~isempty(currentRobotPos)
        plot(currentRobotPos(1), currentRobotPos(2), 'mo', 'MarkerSize', 12, 'LineWidth', 4);
        text(currentRobotPos(1)+15, currentRobotPos(2), 'ROBOT', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
    end
    
    title('Pathfinding A* Real-Time');
    hold off;
    drawnow; % Force UI update
end

clear cam;
disp('Elaborazione terminata.');



% %% 2. Image Capture and Preprocessing
% disp('*** BACKGROUND CAPTURE PHASE ***');
% disp('Please ensure the floor is completely EMPTY (no robot, no cubes).');
% disp('Taking background image in 5 seconds...');
% pause(5);
% emptyFloorImg = snapshot(cam, 'immediate');
% grayEmptyFloor = rgb2gray(emptyFloorImg);
% disp('Background captured successfully.');
% [rows, cols] = size(grayImg);
% 
% % ---------------------------------------------------------
% % Homomorphic Filtering (Suppress Non-uniform Illumination)
% % ---------------------------------------------------------
% grayDouble = im2double(grayImg);
% I_log = log(1 + grayDouble);
% 
% % Create Butterworth High-Pass Filter in Frequency Domain
% sigma = 15; % Cutoff frequency
% [X, Y] = meshgrid(1:cols, 1:rows);
% centerX = ceil(cols/2); centerY = ceil(rows/2);
% D = (X - centerX).^2 + (Y - centerY).^2;
% H = 1 - exp(-(D) ./ (2*sigma^2));
% H = fftshift(H); % Center the filter
% 
% % Apply filter and inverse FFT
% I_fft = fft2(I_log);
% I_filtered_fft = H .* I_fft;
% I_filtered = real(ifft2(I_filtered_fft));
% 
% % Exponential transformation to reverse log
% imgFiltered = exp(I_filtered) - 1;
% imgFiltered = im2uint8(mat2gray(imgFiltered)); 
% 
% % ---------------------------------------------------------
% % Adaptive Thresholding & Morphological Noise Removal
% % ---------------------------------------------------------
% % Calculate local first-order statistics to adapt to floor textures
% T = adaptthresh(imgFiltered, 0.4, 'ForegroundPolarity', 'dark');
% bw = imbinarize(imgFiltered, T);
% 
% % Clean up noise: remove isolated pixels, close gaps, clear borders
% bw = bwmorph(bw, 'clean');
% bw = bwmorph(bw, 'close');
% bw = imclearborder(bw);
% 
% % ---------------------------------------------------------
% % D. ArUco Marker Detection (Spatial Anchors)
% % ---------------------------------------------------------
% % Look for standard ArUco markers to act as coordinate system anchors
% try
%     [markerIDs, markerLocs] = readArucoMarker(img, "DICT_4X4_50");
% catch
%     markerIDs = []; markerLocs = [];
% end
% 
% % ---------------------------------------------------------
% % Rectangular Obstacle Detection
% % ---------------------------------------------------------
% % Measure geometric properties of binary regions
% stats = regionprops(bw, 'BoundingBox', 'Area', 'Extent', 'Centroid');
% 
% rectangularObstacles = [];
% for k = 1:length(stats)
%     % Extent is the ratio of pixels in the region to pixels in the total bounding box.
%     % Rectangular objects have an extent close to 1.
%     if stats(k).Area > 800 && stats(k).Extent > 0.70 
%         rectangularObstacles = [rectangularObstacles; stats(k).BoundingBox];
%     end
% end
% 
% 
% 
% %% 3. ArUco Marker Detection & Occupancy Map Generation
% disp('Acquiring environment map...');
% img = snapshot(cam, 'immediate');
% grayImg = rgb2gray(img);
% 
% % Detect ArUco Markers for precise localization
% [ids, locs] = readArucoMarker(grayImg, markerFamily);
% 
% if isempty(ids) || ~ismember(robotMarkerID, ids) || ~ismember(goalMarkerID, ids)
%     error('Could not find both Robot (ID %d) and Goal (ID %d) ArUco markers!', robotMarkerID, goalMarkerID);
% end
% 
% % Extract Robot Position and Orientation from ArUco
% % Location is represented as a 4x2 table (4 corners, and 2 coordinates for
% % x and y)
% robotIdx = find(ids == robotMarkerID);
% robotCorners = locs(:,:,robotIdx);
% startPos = mean(robotCorners, 1); % Center of the marker
% 
% % Calculate yaw angle based on the top edge of the marker (corners 1 and 2)
% dy = robotCorners(2,2) - robotCorners(1,2);
% dx = robotCorners(2,1) - robotCorners(1,1);
% startYaw = atan2(dy, dx);
% 
% % Extract Goal Position from ArUco
% goalIdx = find(ids == goalMarkerID);
% goalCorners = locs(:,:,goalIdx);
% goalPos = mean(goalCorners, 1);
% 
% % --- Background Subtraction (Map Generation) ---
% % Calculate absolute difference between empty floor and current environment
% diffImg = imabsdiff(grayEmptyFloor, grayImg);
% 
% % Threshold to find added objects (obstacles, robot, goal)
% diffThreshold = 15; % Adjust this value based on your room's lighting (0-255)
% obstacleMask = diffImg > diffThreshold;
% 
% % Clean up noise using morphological operations
% se = strel('disk', 5);
% obstacleMask = imclose(imopen(obstacleMask, se), se);
% 
% % Remove the robot and goal positions from the obstacle mask so A* can find a path
% % (Background subtraction sees them as obstacles too, so we clear a radius around them)
% [X, Y] = meshgrid(1:size(obstacleMask, 2), 1:size(obstacleMask, 1));
% robotClearance = (X - startPos(1)).^2 + (Y - startPos(2)).^2 <= (robotRadius * 2)^2;
% goalClearance = (X - goalPos(1)).^2 + (Y - goalPos(2)).^2 <= (robotRadius * 2)^2;
% obstacleMask(robotClearance | goalClearance) = false;
% 
% % Create a Binary Occupancy Map
% map = binaryOccupancyMap(obstacleMask, mapResolution);
% inflate(map, robotRadius);
% figure;
% show(map);
% 
% %% 4. Path Planning (A* Algorithm)
% disp('Calculating Path...');
% planner = plannerAStarGrid(map);
% % plan() returns the waypoints in [x, y] format
% path = plan(planner, startPos, goalPos);
% 
% if isempty(path)
%     error('No valid path found between robot and goal!');
% end
% 
% % Smooth the path to make it easier for the differential drive to follow
% % (Optional, requires Navigation Toolbox)
% smoothedPath = smoothPathSpline(path); 

% %% 5. Setup Pure Pursuit Controller
% % The Pure Pursuit controller calculates linear/angular velocities 
% % to follow the generated waypoints.
% controller = controllerPurePursuit;
% controller.Waypoints = smoothedPath;
% controller.DesiredLinearVelocity = 0.5; % Set base speed
% controller.MaxAngularVelocity = 1.0;
% controller.LookaheadDistance = 30; % Pixels to look ahead on the path

% %% 6. Real-Time Tracking & Control Loop
% disp('Starting Control Loop...');
% goalReached = false;
% robotCurrentPose = [startPos, 0]; % [x, y, theta]
% 
% while ~goalReached
%     % 1. Capture new frame
%     currentImg = snapshot(cam, 'immediate');
%     curHsv = rgb2hsv(currentImg);
% 
%     % 2. Find Robot's Current Position
%     curRobotMask = (curHsv(:,:,1) > 0.55 & curHsv(:,:,1) < 0.70) & curHsv(:,:,2) > 0.4;
%     curRobotMask = imopen(curRobotMask, se);
%     rProps = regionprops(curRobotMask, 'Centroid', 'Orientation');
% 
%     if ~isempty(rProps)
%         % Update position
%         robotCurrentPose(1:2) = rProps(1).Centroid;
%         % Convert orientation from regionprops to radians. 
%         % (You may need to invert/adjust based on camera angle)
%         robotCurrentPose(3) = deg2rad(-rProps(1).Orientation); 
%     else
%         disp('Tracking lost. Sending stop command.');
%         sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0);
%         continue;
%     end
% 
%     % 3. Calculate Distance to Goal
%     distToGoal = norm(robotCurrentPose(1:2) - goalPos);
%     if distToGoal < 20 % Within 20 pixels of the goal
%         goalReached = true;
%         disp('Goal Reached!');
%         sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0); % Stop
%         break;
%     end
% 
%     % 4. Compute Control Commands (v, omega)
%     [v, omega] = controller(robotCurrentPose);
% 
%     % 5. Send to ESP32 via UDP
%     sendESP32Command(udpSender, esp32_IP, esp32_Port, v, omega);
% 
%     % 6. Visualization
%     subplot(1,2,1);
%     imshow(currentImg); hold on;
%     plot(smoothedPath(:,1), smoothedPath(:,2), 'w-', 'LineWidth', 2);
%     plot(goalPos(1), goalPos(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
%     plot(robotCurrentPose(1), robotCurrentPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
%     title('Live Camera Feed & Tracking');
%     hold off;
% 
%     subplot(1,2,2);
%     show(map); hold on;
%     plot(smoothedPath(:,1), smoothedPath(:,2), 'r-', 'LineWidth', 2);
%     title('Occupancy Map & Path');
%     hold off;
% 
%     drawnow;
%     pause(0.1); % Run loop at roughly 10Hz
% end

% %% Helper Function: Send Data to ESP32
% function sendESP32Command(udpObj, ip, port, linearVel, angularVel)
%     % Create a JSON payload. The ESP32 will parse this easily.
%     % E.g., {"v": 0.5, "w": 0.1}
%     cmdStruct = struct('v', linearVel, 'w', angularVel);
%     jsonStr = jsonencode(cmdStruct);
% 
%     % Send over UDP
%     write(udpObj, uint8(jsonStr), "uint8", ip, port);
% end