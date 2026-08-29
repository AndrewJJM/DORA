% =========================================================================
% Legacy: Pure Pursuit Controller (Not yet integrated)
% =========================================================================
% This module contains the Pure Pursuit path-following controller logic.
% It is kept here for reference and future integration once pathfinding
% is finalized.
%
% To integrate:
%   1. Uncomment the code below
%   2. Add Pure Pursuit config fields to dora_config.m
%   3. Call from the live tracking loop in dora_live.m, using the robot's
%      current pose and the planned path
% =========================================================================

% %% Pure Pursuit Controller Setup
% % The Pure Pursuit controller calculates linear/angular velocities
% % to follow the generated waypoints.
%
% % controller = controllerPurePursuit;
% % controller.Waypoints = smoothedPath;
% % controller.DesiredLinearVelocity = 0.5;  % Base speed
% % controller.MaxAngularVelocity = 1.0;
% % controller.LookaheadDistance = 30;       % Pixels to look ahead
%
% %% Tracking Loop with Pure Pursuit
% % goalReached = false;
% % robotCurrentPose = [startPos, 0];  % [x, y, theta]
% %
% % while ~goalReached
% %     % 1. Capture new frame
% %     currentImg = snapshot(cam, 'immediate');
% %     curHsv = rgb2hsv(currentImg);
% %
% %     % 2. Find Robot's Current Position (HSV-based)
% %     curRobotMask = (curHsv(:,:,1) > 0.55 & curHsv(:,:,1) < 0.70) & curHsv(:,:,2) > 0.4;
% %     curRobotMask = imopen(curRobotMask, se);
% %     rProps = regionprops(curRobotMask, 'Centroid', 'Orientation');
% %
% %     if ~isempty(rProps)
% %         robotCurrentPose(1:2) = rProps(1).Centroid;
% %         robotCurrentPose(3) = deg2rad(-rProps(1).Orientation);
% %     else
% %         disp('Tracking lost. Sending stop command.');
% %         sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0);
% %         continue;
% %     end
% %
% %     % 3. Check distance to goal
% %     distToGoal = norm(robotCurrentPose(1:2) - goalPos);
% %     if distToGoal < 20  % Within 20 pixels
% %         goalReached = true;
% %         disp('Goal Reached!');
% %         sendESP32Command(udpSender, esp32_IP, esp32_Port, 0, 0);
% %         break;
% %     end
% %
% %     % 4. Compute control commands
% %     [v, omega] = controller(robotCurrentPose);
% %
% %     % 5. Send to ESP32
% %     sendESP32Command(udpSender, esp32_IP, esp32_Port, v, omega);
% %
% %     drawnow;
% %     pause(0.1);  % ~10 Hz
% % end
