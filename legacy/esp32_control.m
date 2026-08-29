% =========================================================================
% Legacy: ESP32 UDP Control (Not yet integrated)
% =========================================================================
% This module contains the ESP32 differential-drive control logic.
% It is kept here for reference and future integration once pathfinding
% is finalized.
%
% To integrate:
%   1. Uncomment the function below
%   2. Add ESP32 config fields to dora_config.m
%   3. Call from the live tracking loop in dora_live.m
% =========================================================================

% function sendESP32Command(udpObj, ip, port, linearVel, angularVel)
% %SENDESP32COMMAND  Send velocity commands to ESP32 via UDP.
% %
% %   sendESP32Command(udpObj, ip, port, linearVel, angularVel)
% %
% %   Sends a JSON payload: {"v": <linearVel>, "w": <angularVel>}
% %
% %   Inputs:
% %     udpObj     - UDP object (from udpport)
% %     ip         - ESP32 IP address string
% %     port       - ESP32 UDP port number
% %     linearVel  - desired linear velocity
% %     angularVel - desired angular velocity
%
%     cmdStruct = struct('v', linearVel, 'w', angularVel);
%     jsonStr = jsonencode(cmdStruct);
%     write(udpObj, uint8(jsonStr), "uint8", ip, port);
% end
