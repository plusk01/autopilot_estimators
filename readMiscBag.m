function [imu, state, pose, frame] = readRFBag(bagpath)
%READBAG Extracts messages into matrices without using custom msg defn

% Data is in FRD
frame = 'FRD';

bag = rosbag(bagpath);

% Get IMU messages
bagsel = select(bag, 'Topic', '/pixyrf/rosflight/imu');
msgs = readMessages(bagsel,'DataFormat','struct'); imuMsg = [msgs{:}];
tsec = header([imuMsg.Header]);
tmp = [imuMsg.AngularVelocity]; gX = [tmp.X]; gY = [tmp.Y]; gZ = [tmp.Z];
gyro = [gX' gY' gZ']';
tmp = [imuMsg.LinearAcceleration]; aX = [tmp.X]; aY = [tmp.Y]; aZ = [tmp.Z];
accel = [aX' aY' aZ']';
imu = struct('t',tsec,'gyro',gyro,'accel',accel);

% Get pose messages
truthTopic = '/vicon/pixyrf/pixyrf';
bagsel = select(bag, 'Topic', truthTopic);
msgs = readMessages(bagsel,'DataFormat','struct');
transformStampedMsg = [msgs{:}]; tfMsg = [transformStampedMsg.Transform];
tsec = header([transformStampedMsg.Header]);
tmp = [tfMsg.Translation]; pX = [tmp.X]; pY = [tmp.Y]; pZ = [tmp.Z];
pos = [pX' -pY' -pZ']';
tmp = [tfMsg.Rotation]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' -qY' -qZ']';
pose = struct('t',tsec,'position',pos,'quaternion',quat);
% quat = zeros(4,length(tsec)); quat(1,:) = 1;
% pose = struct('t',tsec,'quaternion',quat);

% Get state messages
bagsel = select(bag, 'Topic', '/pixyrf/attitude');
msgs = readMessages(bagsel,'DataFormat','struct'); stateMsg = [msgs{:}];
tsec = header([stateMsg.Header]);
tmp = [stateMsg.Attitude]; qW=[tmp.W];qX=[tmp.X];qY=[tmp.Y];qZ=[tmp.Z];
quat = [qW' qX' qY' qZ']';
tmp = [stateMsg.AngularVelocity]; wX = [tmp.X]; wY = [tmp.Y]; wZ = [tmp.Z];
omega = [wX' wY' wZ']';
state = struct('t',tsec,'quat',quat,'omega',omega);

% time sync
mint = min([min(state.t), min(pose.t), min(imu.t)]);
state.t = state.t - mint;
pose.t = pose.t - mint;
imu.t = imu.t - mint;

end

function t = header(headerMsg)
stamp = [headerMsg.Stamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end

function t = statestamp(stateMsg)
stamp = [stateMsg.StateStamp];
sec = [stamp.Sec];
nsec = [stamp.Nsec];
t = double(sec) + double(nsec)*1e-9;
end