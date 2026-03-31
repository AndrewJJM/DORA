% =========================================================================
% DORA: Bird's Eye View Pathfinding & ESP32 Control
% =========================================================================
% This script connects to a smartphone camera, extracts a map, identifies
% the robot, goal, and obstacles via color thresholding, computes a path,
% and sends differential drive commands to an ESP32 via UDP.
% =========================================================================

clear; clc; close all;

%% 1. Configuration & Setup
% --- Camera Setup ---
% Using MATLAB Mobile Sensor Support
list = mobiledevlist; %mobile devices connected via matlab cloud
if isempty(list)
    error('No mobile devices found. Please ensure MATLAB Mobile is connected.');
end
m = mobiledev(list.DeviceId{1}); % Access the first device's Id from the table
cam = camera(m);
disp('Connected to Smartphone Camera via MATLAB Cloud.');

% --- ESP32 Communication Setup ---
esp32_IP = '192.168.1.101'; % Replace with your ESP32 IP address
esp32_Port = 4210;          % UDP Port listening on ESP32
udpSender = udpport("IPv4");

% --- Robot & Environment Parameters ---
robotRadius = 20; % Robot radius in pixels (adjust based on camera height)
mapResolution = 1; % 1 pixel per grid cell

%% 2. Initial Image Acquisition & Processing
disp('Acquiring initial environment map...');
img = snapshot(cam, 'immediate');
figure('Name', 'DORA: Pathfinding & Tracking', 'Position', [100, 100, 1000, 500]);

% Convert to HSV for robust color thresholding
hsvImg = rgb2hsv(img);

% Define Color Thresholds (Hue, Saturation, Value)
% Note: You will need to tune these values for your specific lighting/colors!
% Obstacles: Red
obstacleMask = (hsvImg(:,:,1) < 0.05 | hsvImg(:,:,1) > 0.95) & hsvImg(:,:,2) > 0.5 & hsvImg(:,:,3) > 0.2;
% Goal: Green
goalMask = (hsvImg(:,:,1) > 0.25 & hsvImg(:,:,1) < 0.40) & hsvImg(:,:,2) > 0.4 & hsvImg(:,:,3) > 0.2;
% Robot: Blue
robotMask = (hsvImg(:,:,1) > 0.55 & hsvImg(:,:,1) < 0.70) & hsvImg(:,:,2) > 0.4 & hsvImg(:,:,3) > 0.2;

% Clean up masks using morphological operations
se = strel('disk', 5);
obstacleMask = imclose(imopen(obstacleMask, se), se);
goalMask = imclose(imopen(goalMask, se), se);
robotMask = imclose(imopen(robotMask, se), se);

%% 3. Map Generation & Coordinate Extraction
% Create a Binary Occupancy Map from the obstacle mask
map = binaryOccupancyMap(obstacleMask, mapResolution);
% Inflate the map by the robot's radius so it doesn't clip corners
inflate(map, robotRadius);

% Find Robot Starting Position
robotProps = regionprops(robotMask, 'Centroid');
if isempty(robotProps)
    error('Robot not found! Adjust blue color thresholds.');
end
startPos = robotProps(1).Centroid;

% Find Goal Position
goalProps = regionprops(goalMask, 'Centroid');
if isempty(goalProps)
    error('Goal not found! Adjust green color thresholds.');
end
goalPos = goalProps(1).Centroid;

%% 4. Path Planning (A* Algorithm)
disp('Calculating Path...');
planner = plannerAStarGrid(map);
% plan() returns the waypoints in [x, y] format
path = plan(planner, startPos, goalPos);

if isempty(path)
    error('No valid path found between robot and goal!');
end

% Smooth the path to make it easier for the differential drive to follow
% (Optional, requires Navigation Toolbox)
smoothedPath = smoothPathSpline(path); 

%% 5. Setup Pure Pursuit Controller
% The Pure Pursuit controller calculates linear/angular velocities 
% to follow the generated waypoints.
controller = controllerPurePursuit;
controller.Waypoints = smoothedPath;
controller.DesiredLinearVelocity = 0.5; % Set base speed
controller.MaxAngularVelocity = 1.0;
controller.LookaheadDistance = 30; % Pixels to look ahead on the path

%% 6. Real-Time Tracking & Control Loop
disp('Starting Control Loop...');
goalReached = false;
robotCurrentPose = [startPos, 0]; % [x, y, theta]

while ~goalReached
    % 1. Capture new frame
    currentImg = snapshot(cam, 'immediate');
    curHsv = rgb2hsv(currentImg);
    
    % 2. Find Robot's Current Position
    curRobotMask = (curHsv(:,:,1) > 0.55 & curHsv(:,:,1) < 0.70) & curHsv(:,:,2) > 0.4;
    curRobotMask = imopen(curRobotMask, se);
    rProps = regionprops(curRobotMask, 'Centroid', 'Orientation');
    
    if ~isempty(rProps)
        % Update position
        robotCurrentPose(1:2) = rProps(1).Centroid;
        % Convert orientation from regionprops to radians. 
        % (You may need to invert/adjust based on camera angle)
        robotCurrentPose(3) = deg2rad(-rProps(1).Orientation); 
    else
        disp('Tracking lost. Sending stop command.');
        sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0);
        continue;
    end
    
    % 3. Calculate Distance to Goal
    distToGoal = norm(robotCurrentPose(1:2) - goalPos);
    if distToGoal < 20 % Within 20 pixels of the goal
        goalReached = true;
        disp('Goal Reached!');
        sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0); % Stop
        break;
    end
    
    % 4. Compute Control Commands (v, omega)
    [v, omega] = controller(robotCurrentPose);
    
    % 5. Send to ESP32 via UDP
    sendESP32Command(udpSender, esp32_IP, esp32_Port, v, omega);
    
    % 6. Visualization
    subplot(1,2,1);
    imshow(currentImg); hold on;
    plot(smoothedPath(:,1), smoothedPath(:,2), 'w-', 'LineWidth', 2);
    plot(goalPos(1), goalPos(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
    plot(robotCurrentPose(1), robotCurrentPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);
    title('Live Camera Feed & Tracking');
    hold off;
    
    subplot(1,2,2);
    show(map); hold on;
    plot(smoothedPath(:,1), smoothedPath(:,2), 'r-', 'LineWidth', 2);
    title('Occupancy Map & Path');
    hold off;
    
    drawnow;
    pause(0.1); % Run loop at roughly 10Hz
end

%% Helper Function: Send Data to ESP32
function sendESP32Command(udpObj, ip, port, linearVel, angularVel)
    % Create a JSON payload. The ESP32 will parse this easily.
    % E.g., {"v": 0.5, "w": 0.1}
    cmdStruct = struct('v', linearVel, 'w', angularVel);
    jsonStr = jsonencode(cmdStruct);
    
    % Send over UDP
    write(udpObj, uint8(jsonStr), "uint8", ip, port);
end